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
constexpr float kVoltageLimit = 2.8f;
constexpr float kPowerSupplyVoltage = 8.0f;
constexpr float kInitFocVoltage = 0.4f;

// Hardware Pin Configuration
constexpr GpioEntry kDrvEnPin = GPIO_ENTRY(M1_EN_GPIO_Port, M1_EN_Pin, true);
constexpr GpioEntry kDrvCalPin = GPIO_ENTRY(M1_CAL_GPIO_Port, M1_CAL_Pin, true);

// LQR Controller Parameters
// u = K1*theta + K2*theta_dot + K3*omega_w
// These gains are derived from scripts/compute_lqr_gains.py
// They are intended to be used directly: u = kK1*theta + kK2*theta_dot +
// kK3*omega_w Note: The script returned K such that u = -Kx, where K was
// negative. So u = -(-|K1|*theta) = +|K1|*theta. But we need negative feedback
// (u should oppose theta). Given "positive current -> positive tilt", we want u
// = -|K1|*theta.
constexpr float lqr_mat[4] = {-120.8937f, -4.5951f, -0.0058f, 0.1051f};
constexpr float kK1 = lqr_mat[0];
constexpr float kK2 = lqr_mat[1];
constexpr float kK3 = lqr_mat[2];
constexpr float kK4 = lqr_mat[3];

// Safety Thresholds
constexpr float kMaxAngleDeg = 10.0f;
constexpr float kMaxWheelRpm = 1000.0f;
constexpr float kMaxWheelRadS = kMaxWheelRpm * 2.0f * _PI / 60.0f;
constexpr float kStandAngleDeg = 3.0f;
constexpr uint32_t kStandStillTimeMs = 2000;

enum class ControlState { DISABLED, WAITING_FOR_STAND, ENABLED };

static ControlState g_control_state = ControlState::WAITING_FOR_STAND;
static uint32_t g_stand_still_start_ms = 0;
static float g_int_wheel_velocity = 0;

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
        "MPU6050: Accel[%d %d %d]mg Gyro[%d %d %d]md/s last_error: %u\n",
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

void UpdateControlLoop() {
  if (!g_mpu6050 || !motor || !async_spi1) return;

  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  g_mpu6050->GetAccel(ax, ay, az);
  g_mpu6050->GetGyro(gx, gy, gz);

  // Kalman Filter Update
  uint32_t now = HAL_GetTick();
  if (last_kalman_tick == 0) last_kalman_tick = now;
  float dt = (now - last_kalman_tick) / 1000.0f;
  last_kalman_tick = now;

  if (dt <= 0) return;

  // X is downward (reports -1g when vertical), Y is right.
  float accel_tilt_deg = atan2f(ay, -ax) * 180.0f / M_PI;
  // Z is the axis of rotation.
  float tilt_deg = kalman_filter.update(accel_tilt_deg, gz, dt);
  float tilt_rate_deg_s = kalman_filter.getRate();

  float wheel_velocity_rad_s = async_spi1->getVelocity();

  // Safety Checks
  bool angle_out_of_range = std::abs(tilt_deg) > kMaxAngleDeg;
  bool wheel_speed_too_high = std::abs(wheel_velocity_rad_s) > kMaxWheelRadS;

  if (g_control_state == ControlState::ENABLED) {
    if (angle_out_of_range || wheel_speed_too_high) {
      g_control_state = ControlState::WAITING_FOR_STAND;
      motor->target = 0;
      g_int_wheel_velocity = 0;
      std::printf(
          "Controller DISABLED: %s%s (Angle: %d mrad, Speed: %d mrad/s)\n",
          angle_out_of_range ? "Angle out of range " : "",
          wheel_speed_too_high ? "Wheel speed too high" : "",
          static_cast<int>(tilt_deg * 1000.0f),
          static_cast<int>(wheel_velocity_rad_s * 1000.0f));
    } else {
      // Update integrator
      g_int_wheel_velocity += wheel_velocity_rad_s * dt;
      // Anti-windup (cap at value that could produce 10A of current)
      g_int_wheel_velocity =
          _constrain(g_int_wheel_velocity, -2000.0f, 2000.0f);

      // LQR Control Law: u = K1*theta + K2*theta_dot + K3*omega_w +
      // K4*int_omega_w Convert state to radians/SI units
      float theta_rad = tilt_deg * _PI / 180.0f;
      float theta_dot_rad_s = tilt_rate_deg_s * _PI / 180.0f;

      auto u1 = kK1 * theta_rad;
      auto u2 = kK2 * theta_dot_rad_s;
      auto u3 = kK3 * wheel_velocity_rad_s;
      auto u4 = kK4 * g_int_wheel_velocity;

      float u = u1 + u2 + u3 + u4;

      // Apply current limit
      u = _constrain(u, -kCurrentLimit, kCurrentLimit);
      motor->target = u;
      std::printf(
          "%d (theta) + %d (theta_dot) + %d (wheel_vel) + %d (wheel_vel_int)= "
          "%d\n",
          static_cast<int>(u1 * 1000.0f), static_cast<int>(u2 * 1000.0f),
          static_cast<int>(u3 * 1000.0f), static_cast<int>(u4 * 1000.0f),
          static_cast<int>(u * 1000.0f));
    }
  } else if (g_control_state == ControlState::WAITING_FOR_STAND) {
    motor->target = 0;
    if (std::abs(tilt_deg) < kStandAngleDeg) {
      if (g_stand_still_start_ms == 0) {
        g_stand_still_start_ms = now;
      } else if (now - g_stand_still_start_ms > kStandStillTimeMs) {
        g_control_state = ControlState::ENABLED;
        g_stand_still_start_ms = 0;
        g_int_wheel_velocity = 0;
        std::printf("Controller ENABLED: Standing detected.\n");
      }
    } else {
      g_stand_still_start_ms = 0;
    }
  }
}

void Loop() {
  uint32_t start_time = HAL_GetTick();
  uint32_t last_print_time = 0;
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

    if (g_mpu6050) {
      UpdateControlLoop();
    }

    if (HAL_GetTick() - last_print_time >= 250) {
      LogMiscellaneousData();
      std::printf("State: %s Target: %d mA IntWheel: %d rad\n",
                  g_control_state == ControlState::ENABLED ? "ENABLED"
                  : g_control_state == ControlState::WAITING_FOR_STAND
                      ? "WAITING"
                      : "DISABLED",
                  static_cast<int>(1000.f * motor->target),
                  static_cast<int>(g_int_wheel_velocity));
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
