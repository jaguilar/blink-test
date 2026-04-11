/**
 * Hardware Setup Requirements for SPI Sensor HIL Test:
 * ---------------------------------------------------
 * This test validates the AsyncTimerAS5048ASpi driver by using the SPI3
 * peripheral as a hardware slave to simulate the AS5048A sensor.
 *
 * Physical Jumper Wirings (PAx: SPI1 Master, PBx/PCx: SPI3 Slave):
 * - PA5 (SPI1_SCK)  <-> PC10 (SPI3_SCK)
 * - PA6 (SPI1_MISO) <-> PC11 (SPI3_MISO)
 * - PA7 (SPI1_MOSI) <-> PB5  (SPI3_MOSI)
 *
 * Ensure the target is an STM32G474 and the above pins are cross-connected
 * before running the test in --hil mode.
 */

#include <cstdio>
#include <cmath>

#include "CppUTest/TestHarness.h"
#include "foc_types.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_tim.h"

extern "C" {
#include "main.h"
}

using namespace stfoc;

static volatile uint32_t spi3_irq_hits = 0;
static volatile uint32_t dma1_ch8_hits = 0;
static volatile uint16_t spi3_last_rx = 0;
static volatile uint16_t spi3_next_tx = 0;
static volatile uint16_t spi3_rx_history[2] = {0};

extern "C" void SPI3_IRQHandler() {
    if (LL_SPI_IsActiveFlag_RXNE(SPI3)) {
        uint16_t rx = LL_SPI_ReceiveData16(SPI3);
        if (spi3_irq_hits < 2) {
            spi3_rx_history[spi3_irq_hits] = rx;
        }
        spi3_last_rx = rx;
        spi3_irq_hits = spi3_irq_hits + 1;
        LL_SPI_TransmitData16(SPI3, spi3_next_tx);
    }
    if (LL_SPI_IsActiveFlag_OVR(SPI3)) {
        LL_SPI_ClearFlag_OVR(SPI3);
    }
}

// Global instances for DMA completion
volatile bool test_spi_dma_complete = false;

extern "C" void DMA1_Channel8_IRQHandler() {
    dma1_ch8_hits = dma1_ch8_hits + 1;
    if (LL_DMA_IsActiveFlag_TC8(DMA1)) {
        LL_DMA_ClearFlag_TC8(DMA1);
        test_spi_dma_complete = true;
    }
}

TEST_GROUP(SpiSensorTest) {
    void setup() {
        spi3_irq_hits = 0;
        dma1_ch8_hits = 0;
        spi3_rx_history[0] = 0;
        spi3_rx_history[1] = 0;

        printf("--- setup: Configuring Pins for Hardware SPI3 Slave ---\n");
        // Enable clocks
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

        LL_GPIO_InitTypeDef gpio_init = {0};
        gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
        gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init.Pull = LL_GPIO_PULL_NO;
        gpio_init.Alternate = LL_GPIO_AF_6;

        // Configure Slave Pins: PC10 (SCK), PC11 (MISO), PB5 (MOSI)
        gpio_init.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
        LL_GPIO_Init(GPIOC, &gpio_init);
        gpio_init.Pin = LL_GPIO_PIN_5;
        LL_GPIO_Init(GPIOB, &gpio_init);

        // Configure SPI1 Pins: PA5 (SCK), PA6 (MISO), PA7 (MOSI)
        gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
        gpio_init.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
        gpio_init.Alternate = LL_GPIO_AF_5;
        LL_GPIO_Init(GPIOA, &gpio_init);

        // Configure CSn Pin: PA15 (TIM2_CH1)
        gpio_init.Pin = LL_GPIO_PIN_15;
        gpio_init.Alternate = LL_GPIO_AF_1;
        LL_GPIO_Init(GPIOA, &gpio_init);

        stfoc::internal::ResetAllTimers();
    }

    void teardown() {
        printf("--- teardown: Restoring Pins ---\n");
        stfoc::internal::ResetAllTimers();
    }
};

TEST(SpiSensorTest, VerifyAsyncReadWithMotorUpdate) {
    printf("Starting VerifyAsyncReadWithMotorUpdate (Hardware Slave SPI3)\n");
    
    static constexpr StTimerMotorConfig motor_config = {
        .timer_base = TIM1_BASE,
        .pwm_freq = 20000,
        .min_dead_time_nanos = 1000,
    };
    StTimerMotorDriver<motor_config> motor;
    motor.init();
    
    // 2. Async SPI Config
    static constexpr AsyncTimerSpiConfig spi_config = {
        .spi_base = SPI1_BASE,
        .spi_rx_dma_base = DMA1_BASE,
        .spi_rx_dma_channel = LL_DMA_CHANNEL_8,
        .spi_tx_dma_base = DMA1_BASE,
        .spi_tx_dma_channel = LL_DMA_CHANNEL_7,
        .timer_base = TIM2_BASE,
        .timer_channel_csn = LL_TIM_CHANNEL_CH1,
        .spi_baud_rate = LL_SPI_BAUDRATEPRESCALER_DIV256,
    };
    AsyncTimerAS5048ASpi<spi_config> sensor;
    sensor.init();
    
    LL_SPI_Disable(SPI3);
    LL_SPI_InitTypeDef slave_init = {0};
    slave_init.TransferDirection = LL_SPI_FULL_DUPLEX;
    slave_init.Mode = LL_SPI_MODE_SLAVE;
    slave_init.DataWidth = LL_SPI_DATAWIDTH_16BIT;
    slave_init.ClockPolarity = LL_SPI_POLARITY_LOW;
    slave_init.ClockPhase = LL_SPI_PHASE_2EDGE; // Mode 1
    slave_init.NSS = LL_SPI_NSS_SOFT;
    slave_init.BitOrder = LL_SPI_MSB_FIRST;
    LL_SPI_Init(SPI3, &slave_init);
    LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
    
    LL_SPI_Enable(SPI3);

    // Prepare slave response
    spi3_irq_hits = 0;
    spi3_last_rx = 0;
    spi3_next_tx = 0x5678; // Some distinctive value
    LL_SPI_TransmitData16(SPI3, 0x1234); // First response
    LL_SPI_EnableIT_RXNE(SPI3);
    NVIC_SetPriority(SPI3_IRQn, 0);
    NVIC_EnableIRQ(SPI3_IRQn);

    printf("SPI1 CR1: 0x%lx, CR2: 0x%lx | SPI3 CR1: 0x%lx, CR2: 0x%lx\n", 
           (unsigned long)SPI1->CR1, (unsigned long)SPI1->CR2, (unsigned long)SPI3->CR1, (unsigned long)SPI3->CR2);
    printf("DMA1 CH7 CCR: 0x%lx, CH8 CCR: 0x%lx\n",
           (unsigned long)DMA1_Channel7->CCR, (unsigned long)DMA1_Channel8->CCR);

    test_spi_dma_complete = false;
    sensor.AsyncReadFromMotorUpdate<TIM1_BASE>();
    
    // Enable DMA IRQ
    NVIC_SetPriority(DMA1_Channel8_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel8_IRQn);

    printf("Starting motor timer...\n");
    LL_TIM_EnableCounter(TIM1);

    printf("Waiting for SPI DMA completion...\n");
    uint32_t timeout = 2000000;
    while (!test_spi_dma_complete && --timeout);
    
    printf("SPI3 hits: %lu, DMA1 hits: %lu\n", (unsigned long)spi3_irq_hits, (unsigned long)dma1_ch8_hits);
    printf("Slave RX History: 0x%04x 0x%04x\n", spi3_rx_history[0], spi3_rx_history[1]);
    
    sensor.update();
    
    float actual_angle = sensor.getSensorAngle();
    const uint16_t* rx_buf = sensor.getRawRxBuf();
    printf("Raw RX Buffer: 0x%04x 0x%04x\n", rx_buf[0], rx_buf[1]);
    printf("Read Angle: %f rad (%d deg)\n", actual_angle, static_cast<int>(360.f * actual_angle / (2.f * (float)M_PI)));

    LONGS_EQUAL(0x1234, rx_buf[0]);
    LONGS_EQUAL(0x5678, rx_buf[1]);

    DOUBLES_EQUAL(2.205864, actual_angle, 0.00001);

    LONGS_EQUAL(2, spi3_irq_hits);
    LONGS_EQUAL(0xFFFF, spi3_rx_history[0]);
}
