#include "app.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <span>
#include <type_traits>

#include "as5048a_spi_sensor.h"
#include "cmsis_os2.h"
#include "foc_types.h"
#include "spi.h"
#include "stm32_motor_driver.h"
#include "stm32g474xx.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmamux.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "tim.h"
#include "uart_dma.h"

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  EnsureCycleCounterEnabled();
  CheckClockFrequency();
  UartDma_Init();

  std::printf("\n--- Blink Test Startup ---\n");
  std::printf("Initializing Motor Driver...\n");
  motor1_driver.init();
  motor1_driver.setPwm(0.25f, 0.5f, 0.75f);

  std::printf("Initializing AS5048A Sensor...\n");
  async_spi1.init();

  std::printf("Starting Motor Timer and Enabling Driver...\n");
  std::printf("Starting Motor Timer and Enabling Driver...\n");
  LL_TIM_EnableCounter(TIM1);
  motor1_driver.enable();
  std::printf("Startup Complete. Entering Loop.\n");

  // Start the first sensor measurement cycle.
  async_spi1.AsyncReadFromMotorUpdate<TIM1_BASE>();
}

void Loop() {
  uint32_t ticks = osKernelGetTickCount();
  while (true) {
    std::printf("Pos: %ld mrad, Vel: %ld mrad/s\n",
                static_cast<long>(async_spi1.getAngle() * 1000.0f),
                static_cast<long>(async_spi1.getVelocity() * 1000.0f));
    ticks += 1000;
    osDelayUntil(ticks);
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

    // Retrigger the sensor measurement cycle for the next motor update.
    async_spi1.AsyncReadFromMotorUpdate<TIM1_BASE>();
  } else if (LL_DMA_IsActiveFlag_TE8(DMA1)) {
    LL_DMA_ClearFlag_TE8(DMA1);
    spi_dma_error = true;
  }
}

}  // extern "C"
