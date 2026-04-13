#include <cstdio>
#include <cmath>

#include "CppUTest/TestHarness.h"
#include "foc_types.h"
#include "stm32_motor_driver.h"
#include "stm32_adc_current_sense.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_rcc.h"

extern "C" {
#include "main.h"
}

using namespace stfoc;

TEST_GROUP(AdcCurrentSenseTest) {
    void setup() {
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
        
        // PA5 is connected to ADC2_IN13 and DAC1_OUT2 (physical loopback or internal)
        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ANALOG);

        stfoc::internal::ResetAllTimers();
    }

    void teardown() {
        stfoc::internal::ResetAllTimers();
        DAC1->CR &= ~(DAC_CR_EN1 | DAC_CR_EN2);
    }

    void InitDAC1(uint16_t val1, uint16_t val2) {
        DAC1->MCR &= ~(0x3UL << 14);
        DAC1->MCR |=  (0x1UL << 14); // HFSEL
        
        // Mode 1: External + Internal, Buffered (Required for reliable signal strength on PA5)
        DAC1->MCR &= ~(0x7UL << 0);
        DAC1->MCR |= (0x1UL << 0);
        DAC1->MCR &= ~(0x7UL << 16);
        DAC1->MCR |= (0x1UL << 16);

        DAC1->CR |= (DAC_CR_EN1 | DAC_CR_EN2);
        for(volatile int i=0; i<1000; i++);

        DAC1->DHR12R1 = val1;
        DAC1->DHR12R2 = val2;
    }
};

TEST(AdcCurrentSenseTest, ValidateDriverTriggered) {
    printf("Starting ADC Current Sense Driver Validation (Triggered DMA)\n");

    // 1. Motor Timer config (Trigger source)
    static constexpr StTimerMotorConfig motor_config = {
        .timer_base = TIM1_BASE,
        .pwm_freq = 20000,
        .min_dead_time_nanos = 1000,
    };
    StTimerMotorDriver<motor_config> motor;
    motor.init();
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);

    // 2. ADC Driver config
    // ADC1: VREFINT (~1.21V -> ~1500 LSB)
    // ADC2: CH13 (DAC1_OUT2 on PA5)
    static constexpr Stm32AdcCurrentSenseConfig adc_config = {
        .adc1_base = ADC1_BASE,
        .adc1_channel = LL_ADC_CHANNEL_VREFINT,
        .adc2_base = ADC2_BASE,
        .adc2_channel = LL_ADC_CHANNEL_13,
        .dma_base = DMA1_BASE,
        .adc1_dma_channel = LL_DMA_CHANNEL_1,
        .adc2_dma_channel = LL_DMA_CHANNEL_2,
        .shunt_resistance = 0.007f, // 7 milliohms
        .amp_gain = 1.0f,           // 1:1 for DAC direct loopback
        .v_ref = 3.3f,
        .offset_a = 0.0f,
        .offset_b = 0.0f,
        .sampling_time = LL_ADC_SAMPLINGTIME_640CYCLES_5,
    };
    static Stm32AdcCurrentSense<adc_config> sense;
    sense.init();
    sense.SlaveToTimerUpdate<TIM1_BASE>();
    sense.EnableTrigger();

    LL_TIM_EnableCounter(TIM1);
    
    // Test multiple DAC voltages on ADC2, while ADC1 should remain fixed at VREFINT
    uint16_t dac_vals[] = { 1000, 2000, 3000 };
    for (uint16_t v : dac_vals) {
        InitDAC1(v, v);
        // Wait for DMA updates and settling
        for(volatile int i=0; i<500000; i++);
        
        auto currents = sense.getPhaseCurrents();
        uint16_t r1 = *sense.GetAdc1Buffer();
        uint16_t r2 = *sense.GetAdc2Buffer();
        
        printf("  DAC=%4d -> ADC1(VREFINT)=%d, ADC2(CH13)=%d -> Ia=%.3f, Ib=%.3f\n", 
               v, r1, r2, currents.a, currents.b);

        // Basic sanity checks
        CHECK(r1 > 1400 && r1 < 1600); // VREFINT range
        CHECK(std::abs((int)r2 - (int)v) < 300); // DAC matching on ADC2
    }
}
