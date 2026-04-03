#include "app.h"

#include <cassert>
#include <cmath>
#include <span>
#include <type_traits>

#include "cmsis_os2.h"
#include "foc_types.h"
#include "stm32g474xx.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmamux.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"

namespace blink {
namespace {

using namespace stfoc;

volatile bool spi_dma_complete = false;
volatile bool spi_dma_error = false;

constexpr uint32_t kClockFrequencyHz = 160000000;

void EnsureCycleCounterEnabled() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void CheckClockFrequency() {
  LL_RCC_ClocksTypeDef clocks;
  LL_RCC_GetSystemClocksFreq(&clocks);
  assert(clocks.PCLK1_Frequency == kClockFrequencyHz);
}

constexpr auto GetMotor1Config() {
  StTimerMotorConfig config = {
      .timer_base = TIM1_BASE,
      .pwm_freq = 20'000,
      .min_dead_time_nanos = 1000,
  };
  return config;
}
StTimerMotorDriver<GetMotor1Config()> motor1_driver;

constexpr auto GetAsyncSpi1Config() {
  AsyncTimerSpiConfig config = {
      .spi_base = SPI1_BASE,
      .spi_rx_dma_base = DMA1_BASE,
      .spi_rx_dma_channel = LL_DMA_CHANNEL_8,
      .spi_tx_dma_base = DMA1_BASE,
      .spi_tx_dma_channel = LL_DMA_CHANNEL_7,
      .timer_base = TIM2_BASE,
      .timer_channel_csn = LL_TIM_CHANNEL_CH1,
  };
  return config;
}
AsyncTimerAS5048ASpi<GetAsyncSpi1Config()> async_spi1;

}  // namespace
}  // namespace blink

using namespace blink;

extern "C" {

void Setup() {
  EnsureCycleCounterEnabled();
  CheckClockFrequency();

  motor1_driver.init();
  motor1_driver.setPwm(0.25f, 0.5f, 0.75f);

  async_spi1.init();

  motor1_driver.enable();
}

void Loop() {
  while (true) {
    spi_dma_complete = false;

    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
    async_spi1.AsyncReadFromMotorUpdate<GetMotor1Config().timer_base>();

    // Busy wait.
    while (!spi_dma_complete) {
    }
    assert(!spi_dma_error);
  }
}

// The DMA completion handler for the SPI rx channel.
void DMA1_Channel8_IRQHandler() {
  if (LL_DMA_IsActiveFlag_TC8(DMA1)) [[likely]] {
    // Prevent further CSn assert DMAs based on this SPI transaction. Note that
    // we'll also set OPM below to turn off the timer after this cycle, but if
    // we miss that it's fine since there will be no CSn assert and nothing in
    // the SPI DMAs. (We would just do a noop SPI DMA request and a noop
    // deassert of CSn).
    async_spi1.update();
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
    LL_DMA_ClearFlag_TC8(DMA1);
    spi_dma_complete = true;
  } else if (LL_DMA_IsActiveFlag_TE8(DMA1)) {
    LL_DMA_ClearFlag_TE8(DMA1);
    spi_dma_error = true;
  }
}

}  // extern "C"
