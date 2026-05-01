#include "app.h"

#include <atomic>
#include <cassert>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <new>  // Required for placement new

#include "BLDCMotor.h"
#include "STM32G4CORDICTrigFunctions.h"
#include "as5048a_spi_sensor.h"
#include "cmsis_os2.h"
#include "common/base_classes/FOCMotor.h"
#include "common/foc_utils.h"
#include "communication/SimpleFOCDebug.h"
#include "gpio.h"
#include "i2c.h"
#include "kalman_filter.h"
#include "main.h"
#include "mpu6050.h"
#include "spi.h"
#include "stm32_adc_current_sense.h"
#include "stm32_motor_driver.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio_ex.h"
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

extern UART_HandleTypeDef hlpuart1;

// Motor Configuration Constants
constexpr float kMotorTarget = 0.6f;
constexpr MotionControlType kMotionController = MotionControlType::torque;
constexpr TorqueControlType kTorqueController = TorqueControlType::foc_current;
constexpr float kCurrentLimit = 8.f;
constexpr float kVoltageLimit = 2.5f;
constexpr float kPowerSupplyVoltage = 8.0f;
constexpr float kInitFocVoltage = 0.6f;

// Hardware Pin Configuration
constexpr GpioEntry kDrvEnPin = GPIO_ENTRY(M1_EN_GPIO_Port, M1_EN_Pin, true);
constexpr GpioEntry kDrvCalPin = GPIO_ENTRY(M1_CAL_GPIO_Port, M1_CAL_Pin, true);

PerfCounter foc_perf;
PerfCounter int_perf;
bool is_sleeping = false;

void EnsureCycleCounterEnabled() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

constexpr auto GetMotor1Config() {
  constexpr auto pwm_freq = 20'000;
  StTimerMotorConfig config = {
      .timer_base = TIM1_BASE,
      .drv_en = kDrvEnPin,
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
      .timer_base = TIM15_BASE,
      .timer_channel_csn = LL_TIM_CHANNEL_CH2,
      .csn_pin = GPIO_ENTRY(ENC_CSN_GPIO_Port, ENC_CSN_Pin, false),
  };
  return config;
}

constexpr auto GetAdcConfig() {
  Stm32AdcCurrentSenseConfig config = {
      .adc1_base = ADC1_BASE,
      .adc1_channel = LL_ADC_CHANNEL_1,
      .adc2_base = ADC2_BASE,
      .adc2_channel = LL_ADC_CHANNEL_2,
      .dma_base = DMA1_BASE,
      .adc1_dma_channel = LL_DMA_CHANNEL_5,
      .adc2_dma_channel = LL_DMA_CHANNEL_6,
      .shunt_resistance = 0.007f,  // 7 mOhm
      .amp_gain = 10.0f,           // 10x gain
      .v_ref = 3.3f,
      .en_pin = kDrvEnPin,
      .cal_pin = kDrvCalPin,
  };
  return config;
}

typedef StTimerMotorDriver<GetMotor1Config()> MotorDriverInst;
typedef AsyncTimerAS5048ASpi<GetAsyncSpi1Config()> SensorInst;
typedef Stm32AdcCurrentSense<GetAdcConfig()> CurrentSenseInst;

alignas(MotorDriverInst) static uint8_t driver_buf[sizeof(MotorDriverInst)];
alignas(SensorInst) static uint8_t sensor_buf[sizeof(SensorInst)];
alignas(BLDCMotor) static uint8_t motor_buf[sizeof(BLDCMotor)];
alignas(CurrentSenseInst) static uint8_t
    current_sense_buf[sizeof(CurrentSenseInst)];

static MotorDriverInst* motor1_driver = nullptr;
static SensorInst* async_spi1 = nullptr;
static BLDCMotor* motor = nullptr;
static CurrentSenseInst* current_sense = nullptr;

static KalmanFilter kalman_filter;
static uint32_t last_kalman_tick = 0;

extern "C" {

void EnterSleepMode() {
  std::printf("Entering standby mode. PC13 press will wake/reset.\n");
  is_sleeping = true;
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
  if (motor) {
    motor->disable();
  }

  // Clear wakeup flags
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);

  // Enable wakeup pin 2 (PC13) - active high
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);

  // Enter Standby mode. This performs a full reset on wakeup.
  HAL_PWR_EnterSTANDBYMode();

  // Code should not reach here after standby unless entry failed
}

void InitMpu6050() {
  std::printf("[Setup] Initializing MPU6050 via DMA...\n");
  g_imu_event_flags = osEventFlagsNew(nullptr);
  static uint8_t mpu_buf[sizeof(Mpu6050)];
  g_mpu6050 = new (mpu_buf) Mpu6050(&hi2c1, g_imu_event_flags, 0x01);

  if (g_mpu6050->Init()) {
    std::printf("  [OK] MPU6050 initialized.\n");
  } else {
    std::printf("  [FAIL] MPU6050 initialization failed.\n");
  }
}

void Setup() {
  // Wait a couple of seconds so if someone is trying to see early logs they
  // have a chance to connect.
  osDelay(2000);

  EnsureCycleCounterEnabled();

  std::printf("\r\n--- STM32 FOC Recovery Boot ---\r\n");
  osDelay(20);

  UartDma_Init(&hlpuart1);
  osDelay(100);  // Allow early logs to flush

  InitMpu6050();

  // Precision = 3 (12 iterations), Function = Sine (0), Two results (NRES=1)
  // 12 iterations provide ~3.6 decimal digits of precision, meeting the 3-digit
  // requirement.
  CORDIC->CSR = (3 << CORDIC_CSR_PRECISION_Pos) | (0 << CORDIC_CSR_FUNC_Pos) |
                CORDIC_CSR_NRES;

  std::printf("[Setup] Core hardware initialized. DMA logging active.\n");

  std::printf("[Setup] Constructing Driver...\n");
  motor1_driver = new (driver_buf) MotorDriverInst();
  osDelay(10);

  std::printf("[Setup] Constructing Sensor...\n");
  async_spi1 = new (sensor_buf) SensorInst();
  osDelay(10);

  std::printf("[Setup] Constructing Motor...\n");
  motor = new (motor_buf) BLDCMotor(11, 0.040, 380);
  osDelay(20);

  std::printf("[Setup] Constructing Current Sensor...\n");
  current_sense = new (current_sense_buf) CurrentSenseInst();
  osDelay(10);

  // 5. Initializing Objects
  std::printf("[Setup] Initializing Motor Driver...\n");
  motor1_driver->voltage_power_supply = kPowerSupplyVoltage;
  motor1_driver->voltage_limit = 0.9f * motor1_driver->voltage_power_supply;
  motor1_driver->init();
  osDelay(10);

  std::printf("[Setup] Initializing Sensor...\n");
  async_spi1->init();
  osDelay(10);

  std::printf("[Setup] Initializing Current Sensor...\n");
  current_sense->linkDriver(motor1_driver);
  current_sense->init();
  osDelay(10);

  std::printf("[Setup] Configuring Motor...\n");
  motor->voltage_limit =
      std::min(kVoltageLimit, 0.5f * motor1_driver->voltage_limit);
  motor->current_limit = kCurrentLimit;
  motor->controller = kMotionController;
  motor->torque_controller = kTorqueController;
  motor->target = kMotorTarget;
  motor->voltage_sensor_align = kInitFocVoltage;
  motor->zero_electric_angle = 0.58;  // Experimentally verified.
  motor->motion_downsample = 20;
  {
    // Manually set the time intervals to save us the cost of calculating them.
    const float dt = 1.0f / GetMotor1Config().pwm_freq;
    motor->PID_current_q.Ts = dt;
    motor->PID_current_d.Ts = dt;
    motor->LPF_current_q.Ts = dt;
    motor->LPF_current_d.Ts = dt;
    motor->PID_velocity.Ts = dt * (motor->motion_downsample + 1);
    motor->P_angle.Ts = dt * (motor->motion_downsample + 1);
    motor->LPF_velocity.Ts = dt * (motor->motion_downsample + 1);
    motor->LPF_angle.Ts = dt * (motor->motion_downsample + 1);
  }

  std::printf("[Setup] Linking Components...\n");
  motor->linkSensor(async_spi1);
  motor->linkCurrentSense(current_sense);
  std::printf("  [OK] Sensor linked.\n");
  motor->linkDriver(motor1_driver);
  std::printf("  [OK] Driver linked.\n");

  SimpleFOCDebug::enable(&Serial);
  motor->useMonitoring(Serial);
  motor->monitor_downsample = 1;
  motor->monitor_port = &Serial;
  osDelay(20);

  std::printf("[Setup] Initializing Motor...\n");
  motor->init();
  osDelay(10);

  std::printf("[Setup] Starting Hardware Timers...\n");
  async_spi1->AsyncReadFromMotorUpdate<TIM1_BASE>();
  current_sense->SlaveToTimerUpdate<TIM1_BASE>();
  LL_TIM_EnableCounter(TIM1);
  osDelay(10);

  std::printf("[Setup] Initializing FOC Loop...\n");
  motor->initFOC();
  osDelay(10);
  if (motor->motor_status != FOCMotorStatus::motor_ready) {
    std::printf("[Setup] FOC init failed: %d\n", motor->motor_status);
    while (true) {
    }
  }

  motor->phase_resistance = 0.06f;
  motor->axis_inductance.d = motor->axis_inductance.q =
      motor->phase_inductance = 0.000060f;  // 60uH (estimate)

  std::printf("[Setup] Tuning Current Controller...\n");
  motor->tuneCurrentController(500);

  std::printf("[Setup] Final Enable...\n");
  motor->enable();

  std::printf("[Setup] BOOT COMPLETE. Entering loop.\n");
  osDelay(20);

  NVIC_SetPriority(EXTI15_10_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void LogMiscellaneousData() {
  uint32_t mpu_error = 0;
  if (g_mpu6050) {
    mpu_error = g_mpu6050->ConsumeLastError();
  }

  if (motor) {
    motor->monitor();
  }
  if (async_spi1) {
    float angle = async_spi1->getAngle();
    float mech = async_spi1->getMechanicalAngle();
    float velocity = async_spi1->getVelocity();
    const int32_t foc_nanos = foc_perf.ToNanos();
    const int32_t int_nanos = int_perf.ToNanos();

    std::printf(
        "Angle: %d (mrad) Mech: %d (mrad) Velocity: %d (millrad/s) (raw: "
        "%04x) "
        "nFLT:%" PRIu32 " foc_nanos:%" PRId32 " int_nanos:%" PRId32 "\n",
        static_cast<int>(angle * 1000), static_cast<int>(mech * 1000),
        static_cast<int>(velocity * 1000), async_spi1->raw_angle,
        LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_11), foc_nanos, int_nanos);

#if 0
        std::printf(
            "  loopfoc [ns]: total:%d sensor:%d elec:%d torque:%d voltage:%d\n",
            motor->perf_loopfoc.ToNanos(), motor->perf_sensor.ToNanos(),
            motor->perf_elec_angle.ToNanos(), motor->perf_torque.ToNanos(),
            motor->perf_set_voltage.ToNanos());

        std::printf(
            "  move [ns]: total:%d shaft:%d control:%d\n  last_move_us:%d\n",
            motor->perf_move.ToNanos(), motor->perf_shaft.ToNanos(),
            motor->perf_control.ToNanos(), motor->get_last_move_time_us());
#endif
  }
  if (current_sense) {
    auto currents = current_sense->getPhaseCurrents();
    std::printf("Current [mA]: A:%d B:%d C:%d (Offsets: %dmV, %dmV)\n",
                static_cast<int>(currents.a * 1000.0f),
                static_cast<int>(currents.b * 1000.0f),
                static_cast<int>(currents.c * 1000.0f),
                static_cast<int>(current_sense->offset_ia * 1000.0f),
                static_cast<int>(current_sense->offset_ib * 1000.0f));
  }
  if (g_mpu6050) {
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    g_mpu6050->GetAccel(ax, ay, az);
    g_mpu6050->GetGyro(gx, gy, gz);
    std::printf(
        "MPU6050: Accel[%d %d %d]mg Gyro[%d %d %d]md/s last_error: %lu\n",
        static_cast<int>(ax * 1000.0f), static_cast<int>(ay * 1000.0f),
        static_cast<int>(az * 1000.0f), static_cast<int>(gx * 1000.0f),
        static_cast<int>(gy * 1000.0f), static_cast<int>(gz * 1000.0f),
        mpu_error);

    float tilt = kalman_filter.getAngle();
    float velocity = kalman_filter.getRate();

    std::printf("Kalman: Tilt: %d (mdeg) Velocity: %d (mdeg/s)\n",
                static_cast<int>(tilt * 1000.0f),
                static_cast<int>(velocity * 1000.0f));
  }
}

void UpdateKalmanFilter() {
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  g_mpu6050->GetAccel(ax, ay, az);
  g_mpu6050->GetGyro(gx, gy, gz);

  // Kalman Filter Update
  uint32_t now = HAL_GetTick();
  if (last_kalman_tick == 0) last_kalman_tick = now;
  float dt = (now - last_kalman_tick) / 1000.0f;
  last_kalman_tick = now;

  if (dt > 0) {
    // X is downward (reports -1g when vertical), Y is right.
    float accel_tilt = atan2f(ay, -ax) * 180.0f / _PI;
    // Z is the axis of rotation.
    kalman_filter.update(accel_tilt, gz, dt);
  }
}

void CharacterizeInertia() {
  struct Sample {
    float time;
    float angle;
  };
  // Reduced to 500 samples to fit in 80KB RAM (200Hz * 2s = 400 samples).
  static constexpr int kMaxSamples = 500;
  static Sample samples[kMaxSamples];

  if (!motor) return;

  const float current_limit = motor->current_limit;
  const float current_step = current_limit / 10.0f;
  const float target_velocity = 500.0f * _PI / 30.0f;  // 500 RPM in rad/s

  std::printf("current_mA,time_ms,advancement_mrad\n");

  for (float direction : {1.0f, -1.0f}) {
    for (int i = 1; i <= 10; ++i) {
      float target_current = i * current_step * direction;

      // Phase 1: Wait for wheel to stop spinning
      motor->target = 0;
      while (std::abs(motor->shaft_velocity) > 0.1f) {
        osDelay(10);
      }
      osDelay(1000);  // Additional stabilization

      // Phase 2: Start test
      int sample_count = 0;
      uint32_t start_ticks = HAL_GetTick();
      motor->target = target_current;

      while (sample_count < kMaxSamples) {
        uint32_t now = HAL_GetTick();
        float elapsed = (now - start_ticks) / 1000.0f;
        float shaft_angle = motor->shaft_angle;

        samples[sample_count++] = {elapsed, shaft_angle};

        if (elapsed >= 1.0f || std::abs(motor->shaft_velocity) >= target_velocity) {
          break;
        }
        osDelay(5);  // ~200Hz sampling
      }

      motor->target = 0;

      // Phase 3: Print results (discard first 10 and last 10)
      if (sample_count > 20) {
        for (int s = 10; s < sample_count - 10; ++s) {
          long current_mA = (long)(target_current * 1000.0f);
          long time_ms = (long)(samples[s].time * 1000.0f);
          float delta = samples[s].angle - samples[s - 1].angle;
          // Handle wrap-around
          if (delta > _PI) {
            delta -= 2.0f * _PI;
          } else if (delta < -_PI) {
            delta += 2.0f * _PI;
          }
          long advancement_mrad = (long)(delta * 1000.0f);
          std::printf("%ld,%ld,%ld\n", current_mA, time_ms, advancement_mrad);
        }
      }
    }
  }

  while (true) {
    osDelay(1000);
  }
}

void Loop() {
  CharacterizeInertia();
  uint32_t start_time = HAL_GetTick();
  uint32_t last_print_time = 0;
  uint32_t last_dir_change = HAL_GetTick();
  while (true) {
    // Wait for the MPU6050 data-ready event (set on DMA completion)
    // or timeout after 50ms to keep the loop alive for buttons/other tasks.
    if (((Mpu6050::FLAG_DONE | Mpu6050::FLAG_ERROR) &
         osEventFlagsWait(g_imu_event_flags, 0x01, osFlagsWaitAny, 50)) == 0) {
      std::printf("Timed out waiting for MPU6050 interrupt!\n");
    }

    // Check for button press (PC13 active-high) for immediate sleep
    if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13)) {
      EnterSleepMode();
    }

    if (HAL_GetTick() - start_time > 10 * 60 * 1000) {
      EnterSleepMode();
      start_time = HAL_GetTick();
    }

    if (HAL_GetTick() - last_dir_change > 5000) {
      motor->target = -motor->target;
      last_dir_change = HAL_GetTick();
    }

    if (g_mpu6050) {
      UpdateKalmanFilter();
    }

    if (HAL_GetTick() - last_print_time >= 1000) {
      // LogMiscellaneousData();
      last_print_time = HAL_GetTick();
    }
  }
}

void __attribute__((section(".ccmsram"))) DMA1_Channel8_IRQHandler() {
  if (LL_DMA_IsActiveFlag_TC8(DMA1)) [[likely]] {
    auto int_token = int_perf.GetToken();
    LL_DMA_ClearFlag_TC8(DMA1);
    if (async_spi1) {
      async_spi1->DmaComplete();
    }
    if (motor && motor->enabled) {
      auto foc_token = foc_perf.GetToken();
      motor->loopFOC();
      motor->move();
    }
    if (!is_sleeping && current_sense) {
      current_sense->AsyncReadFromMotorUpdate<TIM1_BASE>();
    }
    if (!is_sleeping && async_spi1) {
      async_spi1->AsyncReadFromMotorUpdate<TIM1_BASE>();
    }
  } else if (LL_DMA_IsActiveFlag_TE8(DMA1)) {
    LL_DMA_ClearFlag_TE8(DMA1);
  }
}

}  // extern "C"
