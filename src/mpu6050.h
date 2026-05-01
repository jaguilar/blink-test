#ifndef STFOC_MPU6050_H
#define STFOC_MPU6050_H

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif
#include "cmsis_os2.h"
#include "i2c.h"

#ifdef __cplusplus
namespace stfoc {

class Mpu6050;
extern Mpu6050* g_mpu6050;
extern osEventFlagsId_t g_imu_event_flags;

class Mpu6050 {
 public:
  // Register addresses
  static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
  static constexpr uint8_t REG_CONFIG = 0x1A;
  static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
  static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
  static constexpr uint8_t REG_FIFO_EN = 0x23;
  static constexpr uint8_t REG_INT_ENABLE = 0x38;
  static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
  static constexpr uint8_t REG_TEMP_OUT_H = 0x41;
  static constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
  static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
  static constexpr uint8_t REG_WHO_AM_I = 0x75;

  static constexpr uint8_t I2C_ADDR = 0x68 << 1;  // HAL uses left-shifted address

  static constexpr uint32_t FLAG_DONE = 0x01;
  static constexpr uint32_t FLAG_ERROR = 0x02;

  explicit Mpu6050(I2C_HandleTypeDef* hi2c,
                   osEventFlagsId_t external_event_flags = nullptr,
                   uint32_t external_event_bit = 0);
  ~Mpu6050();

  bool Init();

  // High-level blocking read (uses FinishRead internally)
  bool ReadAccel(float& ax, float& ay, float& az);
  bool ReadGyro(float& gx, float& gy, float& gz);

  // New non-blocking interrupt-driven interface
  void HandleInterrupt();  // Called from ISR
  bool FinishRead();       // Called from task to wait and parse

  void GetAccel(float& ax, float& ay, float& az) const;
  void GetGyro(float& gx, float& gy, float& gz) const;

  // Called from HAL callbacks
  void OnTransferComplete();
  void OnTransferError();

  uint32_t ConsumeLastError() {
    uint32_t err = last_error_;
    last_error_ = 0;
    return err;
  }

 private:
  I2C_HandleTypeDef* hi2c_;
  osEventFlagsId_t event_flags_;
  osEventFlagsId_t external_event_flags_;
  uint32_t external_event_bit_;

  bool is_background_reading_;
  uint8_t dma_buffer_[14];
  float ax_, ay_, az_;
  float gx_, gy_, gz_;
  uint32_t last_error_ = 0;

  void ParseDmaBuffer();
  bool WaitForTransfer(uint32_t timeout_ms = 100);

  bool WriteReg(uint8_t reg, uint8_t value);
  bool ReadReg(uint8_t reg, uint8_t& value);
  bool ReadBytes(uint8_t reg, uint8_t* data, uint16_t size);
  bool StartReadDMA(uint8_t reg, uint8_t* data, uint16_t size);
};

}  // namespace stfoc
#endif

#ifdef __cplusplus
extern "C" {
#endif
// If an MPU6050 instance has been initialized, this calls
// Mpu6050::HandleInterrupt.
void Mpu6050_HandleInterrupt();
#ifdef __cplusplus
}
#endif

#endif  // STFOC_MPU6050_H
