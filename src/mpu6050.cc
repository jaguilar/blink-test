#include "mpu6050.h"
#include <cstdio>

namespace stfoc {

bool Mpu6050::Init() {
  uint8_t who_am_i = 0;
  if (!ReadReg(REG_WHO_AM_I, who_am_i)) {
    std::printf("MPU6050: Failed to read WHO_AM_I\n");
    return false;
  }

  std::printf("MPU6050: WHO_AM_I = 0x%02X\n", who_am_i);
  if (who_am_i != 0x68) {
    std::printf("MPU6050: Unexpected WHO_AM_I value (expected 0x68)\n");
    return false;
  }

  // Wake up the device (set PWR_MGMT_1 to 0)
  if (!WriteReg(REG_PWR_MGMT_1, 0x00)) {
    std::printf("MPU6050: Failed to write PWR_MGMT_1\n");
    return false;
  }

  std::printf("MPU6050: Initialization successful\n");
  return true;
}

bool Mpu6050::ReadAccel(float& ax, float& ay, float& az) {
  uint8_t data[6];
  if (!ReadBytes(REG_ACCEL_XOUT_H, data, 6)) return false;

  int16_t raw_ax = (static_cast<int16_t>(data[0]) << 8) | data[1];
  int16_t raw_ay = (static_cast<int16_t>(data[2]) << 8) | data[3];
  int16_t raw_az = (static_cast<int16_t>(data[4]) << 8) | data[5];

  // Default range is +/- 2g, so 16384 LSB/g
  ax = static_cast<float>(raw_ax) / 16384.0f;
  ay = static_cast<float>(raw_ay) / 16384.0f;
  az = static_cast<float>(raw_az) / 16384.0f;

  return true;
}

bool Mpu6050::ReadGyro(float& gx, float& gy, float& gz) {
  uint8_t data[6];
  if (!ReadBytes(REG_GYRO_XOUT_H, data, 6)) return false;

  int16_t raw_gx = (static_cast<int16_t>(data[0]) << 8) | data[1];
  int16_t raw_gy = (static_cast<int16_t>(data[2]) << 8) | data[3];
  int16_t raw_gz = (static_cast<int16_t>(data[4]) << 8) | data[5];

  // Default range is +/- 250 deg/s, so 131 LSB/(deg/s)
  gx = static_cast<float>(raw_gx) / 131.0f;
  gy = static_cast<float>(raw_gy) / 131.0f;
  gz = static_cast<float>(raw_gz) / 131.0f;

  return true;
}

bool Mpu6050::WriteReg(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c_, I2C_ADDR, data, 2, 100);
  if (status != HAL_OK) {
    std::printf("MPU6050: WriteReg(0x%02X) failed with status %d\n", reg, status);
  }
  return status == HAL_OK;
}

bool Mpu6050::ReadReg(uint8_t reg, uint8_t& value) {
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c_, I2C_ADDR, &reg, 1, 100);
  if (status != HAL_OK) {
    std::printf("MPU6050: ReadReg(0x%02X) transmit failed with status %d\n", reg, status);
    return false;
  }
  status = HAL_I2C_Master_Receive(hi2c_, I2C_ADDR, &value, 1, 100);
  if (status != HAL_OK) {
    std::printf("MPU6050: ReadReg(0x%02X) receive failed with status %d\n", reg, status);
  }
  return status == HAL_OK;
}

bool Mpu6050::ReadBytes(uint8_t reg, uint8_t* data, uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c_, I2C_ADDR, &reg, 1, 100);
  if (status != HAL_OK) {
    std::printf("MPU6050: ReadBytes(0x%02X) transmit failed with status %d\n", reg, status);
    return false;
  }
  status = HAL_I2C_Master_Receive(hi2c_, I2C_ADDR, data, size, 100);
  if (status != HAL_OK) {
    std::printf("MPU6050: ReadBytes(0x%02X) receive failed with status %d\n", reg, status);
  }
  return status == HAL_OK;
}

}  // namespace stfoc
