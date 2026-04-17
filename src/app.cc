#include "app.h"

#include <cmath>
#include <cstdio>
#include <new>  // Required for placement new

#include "BLDCMotor.h"
#include "as5048a_spi_sensor.h"
#include "cmsis_os2.h"
#include "communication/SimpleFOCDebug.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "stm32_motor_driver.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_tim.h"
#include "system_stm32g4xx.h"
#include "tim.h"
#include "uart_dma.h"
#include "usart.h"

using namespace stfoc;

extern UART_HandleTypeDef huart1;

bool is_sleeping = false;

void EnsureCycleCounterEnabled() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

constexpr auto GetMotor1Config() {
  constexpr auto pwm_freq = 20'000;
  StTimerMotorConfig config = {
      .timer_base = TIM1_BASE,
      .drv_en = {GPIOB_BASE, LL_GPIO_PIN_0, true},
      .pwm_freq = pwm_freq,
  };
  return config;
}

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

typedef StTimerMotorDriver<GetMotor1Config()> MotorDriverInst;
typedef AsyncTimerAS5048ASpi<GetAsyncSpi1Config()> SensorInst;

alignas(MotorDriverInst) static uint8_t driver_buf[sizeof(MotorDriverInst)];
alignas(SensorInst) static uint8_t sensor_buf[sizeof(SensorInst)];
alignas(BLDCMotor) static uint8_t motor_buf[sizeof(BLDCMotor)];

static MotorDriverInst* motor1_driver = nullptr;
static SensorInst* async_spi1 = nullptr;
static BLDCMotor* motor = nullptr;

void ExitSleepMode();

void EnterSleepMode() {
  std::printf("Entering sleep mode...\n");
  is_sleeping = true;
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
  LL_TIM_DisableCounter(TIM1);
  LL_TIM_DisableCounter(TIM2);
  while (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13) == 0) {
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  }
  ExitSleepMode();
}

void ExitSleepMode() {
  is_sleeping = false;
  std::printf("Waking up...\n");
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
  LL_TIM_EnableCounter(TIM1);
  LL_TIM_EnableCounter(TIM2);
}

extern "C" {

void Setup() {
  // Wait a couple of seconds so if someone is trying to see early logs they
  // have a chance to connect.
  osDelay(2000);

  // 1. Core Init
  EnsureCycleCounterEnabled();
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  std::printf("\r\n--- STM32 FOC Recovery Boot ---\r\n");
  osDelay(20);

  // 3. Initialize Hardware HAL/LL
  UartDma_Init();
  osDelay(100);  // Allow early logs to flush
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  std::printf("[Setup] Core hardware initialized. DMA logging active.\n");

  // 4. Manual Construction
  std::printf("[Setup] Constructing Driver...\n");
  motor1_driver = new (driver_buf) MotorDriverInst();
  osDelay(10);

  std::printf("[Setup] Constructing Sensor...\n");
  async_spi1 = new (sensor_buf) SensorInst();
  osDelay(10);

  std::printf("[Setup] Constructing Motor...\n");
  motor = new (motor_buf) BLDCMotor(11, 0.040, 380);
  osDelay(20);

  // 5. Initializing Objects
  std::printf("[Setup] Initializing Motor Driver...\n");
  motor1_driver->voltage_power_supply = 16.0f;
  motor1_driver->voltage_limit = 0.9f * motor1_driver->voltage_power_supply;
  motor1_driver->init();
  osDelay(10);

  std::printf("[Setup] Initializing Sensor...\n");
  async_spi1->init();
  osDelay(10);

  std::printf("[Setup] Configuring Motor...\n");
  motor->voltage_limit = 0.5f * motor1_driver->voltage_limit;
  motor->current_limit = 2.0f;
  motor->controller = MotionControlType::velocity_openloop;
  motor->target = .1f;

  std::printf("[Setup] Linking Components...\n");
  motor->linkSensor(async_spi1);
  std::printf("  [OK] Sensor linked.\n");
  motor->linkDriver(motor1_driver);
  std::printf("  [OK] Driver linked.\n");

  SimpleFOCDebug::enable(&Serial);
  motor->useMonitoring(Serial);
  motor->monitor_downsample = 1;
  osDelay(20);

  std::printf("[Setup] Initializing Motor...\n");
  motor->init();
  osDelay(10);

  std::printf("[Setup] Starting Hardware Timers...\n");
  LL_TIM_EnableCounter(TIM1);
  osDelay(10);

  std::printf("[Setup] Initializing FOC Loop...\n");
  motor->monitor_downsample = 1;
  motor->monitor_port = &Serial;
  motor->initFOC();
  osDelay(10);

  std::printf("[Setup] Final Enable...\n");
  motor->enable();

  std::printf("[Setup] BOOT COMPLETE. Entering loop.\n");
  osDelay(20);

  async_spi1->AsyncReadFromMotorUpdate<TIM1_BASE>();

  NVIC_SetPriority(EXTI15_10_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Loop() {
  while (true) {
    if (motor) {
      motor->monitor();
    }
    if (async_spi1) {
      float angle = async_spi1->getAngle();
      float velocity = async_spi1->getVelocity();
      std::printf(
          "Angle: %d (millrad), Velocity: %d (millrad/s) (raw: %04x) "
          "(mech_angle millirads: %04d)\n",
          static_cast<int>(angle * 1000), static_cast<int>(velocity * 1000),
          async_spi1->getRawRxBuf()[1] & 0b00111111'11111111,
          static_cast<int>(async_spi1->getMechanicalAngle() * 1000));
    }
    osDelay(1000);
  }
}

void DMA1_Channel8_IRQHandler() {
  if (LL_DMA_IsActiveFlag_TC8(DMA1)) [[likely]] {
    if (async_spi1) async_spi1->update();
    LL_DMA_ClearFlag_TC8(DMA1);

    if (motor && motor->enabled) {
      motor->loopFOC();
      motor->move();
    }

    if (!is_sleeping && async_spi1) {
      async_spi1->AsyncReadFromMotorUpdate<TIM1_BASE>();
    }
  } else if (LL_DMA_IsActiveFlag_TE8(DMA1)) {
    LL_DMA_ClearFlag_TE8(DMA1);
  }
}

}  // extern "C"
