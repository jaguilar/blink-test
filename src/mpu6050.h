#ifndef STFOC_MPU6050_H
#define STFOC_MPU6050_H

#include <cstdint>
#include "cmsis_os2.h"
#include "i2c.h"

namespace stfoc {

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

  explicit Mpu6050(I2C_HandleTypeDef* hi2c);
  ~Mpu6050();

  bool Init();
  bool ReadAccel(float& ax, float& ay, float& az);
  bool ReadGyro(float& gx, float& gy, float& gz);

  // Called from HAL callbacks
  void OnTransferComplete();
  void OnTransferError();

 private:
  I2C_HandleTypeDef* hi2c_;
  osEventFlagsId_t event_flags_;
  static constexpr uint32_t FLAG_DONE = 0x01;
  static constexpr uint32_t FLAG_ERROR = 0x02;

  bool WaitForTransfer(uint32_t timeout_ms = 100);

  bool WriteReg(uint8_t reg, uint8_t value);
  bool ReadReg(uint8_t reg, uint8_t& value);
  bool ReadBytes(uint8_t reg, uint8_t* data, uint16_t size);
};

}  // namespace stfoc

#endif  // STFOC_MPU6050_H
