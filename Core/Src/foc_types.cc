#include "foc_types.h"

#include <cassert>

#include "common/base_classes/FOCDriver.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"

namespace stfoc {

uint32_t StTimerMotorConfig::ComputeArr() const {
  LL_RCC_ClocksTypeDef rcc_clocks;
  LL_RCC_GetSystemClocksFreq(&rcc_clocks);
  // Note: in center-aligned mode, the period of the timer is 2x the ARR
  // value. Therefore, we divide by 2 here.
  return rcc_clocks.PCLK2_Frequency / pwm_freq / 2;
}

namespace internal {

void InitGpio(const GpioEntry& entry) {
  if (!entry.gpio_base || !entry.pin) return;
  auto gpio = entry.gpio();
  auto pin = entry.pin;
  if (entry.active_high) {
    LL_GPIO_ResetOutputPin(gpio, pin);
  } else {
    LL_GPIO_SetOutputPin(gpio, pin);
  }
  LL_GPIO_SetPinMode(gpio, pin, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(gpio, pin, LL_GPIO_OUTPUT_PUSHPULL);
}

void GpioAssert(const GpioEntry& entry) {
  entry.gpio()->BSRR = entry.active_high ? entry.pin : (entry.pin << 16);
}

void GpioDeassert(const GpioEntry& entry) {
  entry.gpio()->BSRR = entry.active_high ? (entry.pin << 16) : entry.pin;
}

void InitTimer(const StTimerMotorConfig& config) {
  LL_RCC_ClocksTypeDef rcc_clocks;
  LL_RCC_GetSystemClocksFreq(&rcc_clocks);

  LL_TIM_InitTypeDef timer_init = {
      .Prescaler = 0,
      // We don't use any interrupts on the motor channels, so whether we're in
      // up, down, or up-down mode does not matter.
      .CounterMode = LL_TIM_COUNTERMODE_CENTER_UP,
      .Autoreload = config.ComputeArr(),
      .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 1,
  };
  LL_TIM_Init(config.timer(), &timer_init);
  LL_TIM_SetCounter(config.timer(), 0);

  LL_TIM_OC_InitTypeDef oc_init = {
      .OCMode = LL_TIM_OCMODE_PWM2,
      .OCState = LL_TIM_OCSTATE_ENABLE,
      .CompareValue = timer_init.Autoreload,
      .OCPolarity = LL_TIM_OCPOLARITY_HIGH,
      .OCIdleState = LL_TIM_OCIDLESTATE_LOW,
  };
  for (int chan :
       {LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3}) {
    LL_TIM_OC_Init(config.timer(), chan, &oc_init);
    LL_TIM_OC_EnablePreload(config.timer(), chan);
  }
}

void ResetAllTimers() {
  TIM_TypeDef* timers[] = {TIM1, TIM8, TIM20};
  const char* names[] = {"TIM1", "TIM8", "TIM20"};
  for (int i = 0; i < 3; ++i) {
    auto* tim = timers[i];
    auto name = names[i];
    uint32_t smcr_before = tim->SMCR;
    LL_TIM_DisableCounter(tim);
    tim->CR1 = 0;
    tim->CR2 = 0;
    tim->SMCR = 0;
    tim->DIER = 0;
    tim->SR = 0;
    tim->EGR = TIM_EGR_UG;
    tim->CCMR1 = 0;
    tim->CCMR2 = 0;
    tim->CCER = 0;
    tim->CNT = 0;
    tim->PSC = 0;
    tim->ARR = 0xFFFF;
    tim->CCR1 = 0;
    tim->CCR2 = 0;
    tim->CCR3 = 0;
    tim->CCR4 = 0;
    tim->BDTR = 0;
    tim->DCR = 0;
    tim->AF1 = 0;
    tim->AF2 = 0;
    tim->TISEL = 0;
    tim->OR = 0;
    printf("  Reset %s: SMCR 0x%lx -> 0x%lx\n", name, (unsigned long)smcr_before, (unsigned long)tim->SMCR);
  }
}

uint32_t SyncReadSpi(const AsyncTimerSpiConfig& config, uint16_t address) {
  assert(!LL_TIM_IsEnabledCounter(config.tim()) &&
         "Timer must not have been started when SyncReadSpi is called.");
  assert(LL_TIM_GetCounter(config.tim()) == 0 &&
         "Timer counter must be at 0 when SyncReadSpi is called.");
  assert(LL_TIM_OC_GetMode(config.tim(), config.timer_channel_csn) ==
             LL_TIM_OCMODE_PWM2 &&
         "Timer output should be in PWM2 before SyncReadSpi is called.");

  assert(address <= 0x3FFF && "AS5048A addresses are 14 bits");
  uint16_t read_command = address & (1 << 14);  // (bit 14 == 1 for read)
  bool is_odd = std::popcount(read_command) & 1;
  if (is_odd) {
    read_command |= 1 << 15;
  }

  auto busy_wait_ns = [](uint32_t ns) {
    uint32_t start = DWT->CYCCNT;
    uint32_t wait_ticks =
        (1'000'000'000 + SystemCoreClock - 1) / SystemCoreClock * ns;
    while ((uint32_t)(DWT->CYCCNT - start) < wait_ticks) {
    }
  };

  auto csn_set_asserted = [&](bool active) {
    // Note: PWM2 is the mode where if the timer is reset, the output will
    // also be inactive. This is the mode we want to return to at the end of the
    // SyncReadSpi and has the same effect as FORCED_INACTIVE when the timer
    // counter is reset.
    if (!active) busy_wait_ns(50);
    LL_TIM_OC_SetMode(
        config.tim(), config.timer_channel_csn,
        active ? LL_TIM_OCMODE_FORCED_ACTIVE : LL_TIM_OCMODE_PWM2);
    if (active) busy_wait_ns(350);
  };

  while (!LL_SPI_IsActiveFlag_TXE(config.spi())) {
  }

  // In case we were talking to the AS5048A immediately before this, execute the
  // inter-command delay.
  busy_wait_ns(350);
  csn_set_asserted(true);
  LL_SPI_TransmitData16(config.spi(), read_command);
  while (!LL_SPI_IsActiveFlag_TXE(config.spi())) {
  }
  csn_set_asserted(false);

  // Discard the response of the previous command.
  while (LL_SPI_IsActiveFlag_RXNE(config.spi())) {
    LL_SPI_ReceiveData16(config.spi());
  }

  busy_wait_ns(350);  // Inter-command-delay.
  csn_set_asserted(true);
  uint16_t nop_command = 0x0000;
  LL_SPI_TransmitData16(config.spi(), nop_command);
  while (!LL_SPI_IsActiveFlag_RXNE(config.spi())) {
  }
  uint16_t result = LL_SPI_ReceiveData16(config.spi()) & 0x3FFF;
  csn_set_asserted(false);
  return result;
}

uint32_t SpiNRxDmaReq(SPI_TypeDef* spi) {
  if (spi == SPI1) {
    return LL_DMAMUX_REQ_SPI1_RX;
  } else if (spi == SPI2) {
    return LL_DMAMUX_REQ_SPI2_RX;
  } else if (spi == SPI3) {
    return LL_DMAMUX_REQ_SPI3_RX;
  } else if (spi == SPI4) {
    return LL_DMAMUX_REQ_SPI4_RX;
  } else {
    assert(false && "Unsupported SPI instance in SpiNDmaReq");
    return 0;
  }
}

uint32_t TimNCh2DmaReq(TIM_TypeDef* tim) {
  if (tim == TIM1) {
    return LL_DMAMUX_REQ_TIM1_CH2;
  } else if (tim == TIM2) {
    return LL_DMAMUX_REQ_TIM2_CH2;
  } else if (tim == TIM3) {
    return LL_DMAMUX_REQ_TIM3_CH2;
  } else if (tim == TIM4) {
    return LL_DMAMUX_REQ_TIM4_CH2;
  } else if (tim == TIM5) {
    return LL_DMAMUX_REQ_TIM5_CH2;
  } else if (tim == TIM8) {
    return LL_DMAMUX_REQ_TIM8_CH2;
  } else if (tim == TIM20) {
    return LL_DMAMUX_REQ_TIM20_CH2;
  } else {
    assert(false && "Unsupported timer instance in TimNCh2DmaReq");
    return 0;
  }
}

}  // namespace internal

const uint16_t* AS5048ReadAngleCommandBuf() {
  alignas(uint16_t) static const uint16_t buf[] = {0xFFFF, 0x0000};
  return buf;
}

}  // namespace stfoc
