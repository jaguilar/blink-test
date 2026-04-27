#include "mpu6050.h"
#include <cstdio>

namespace stfoc {

static Mpu6050* g_mpu6050_hi2c1 = nullptr;

// Internal callback bridges for HAL_I2C_RegisterCallback
static void I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
  if (hi2c->Instance == I2C1 && g_mpu6050_hi2c1) {
    g_mpu6050_hi2c1->OnTransferComplete();
  }
}

static void I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  if (hi2c->Instance == I2C1 && g_mpu6050_hi2c1) {
    g_mpu6050_hi2c1->OnTransferComplete();
  }
}

static void I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
  if (hi2c->Instance == I2C1 && g_mpu6050_hi2c1) {
    g_mpu6050_hi2c1->OnTransferError();
  }
}

Mpu6050::Mpu6050(I2C_HandleTypeDef* hi2c) : hi2c_(hi2c) {
  event_flags_ = osEventFlagsNew(nullptr);
  if (hi2c_->Instance == I2C1) {
    g_mpu6050_hi2c1 = this;
  }
}

Mpu6050::~Mpu6050() {
  if (hi2c_->Instance == I2C1 && g_mpu6050_hi2c1 == this) {
    g_mpu6050_hi2c1 = nullptr;
  }
  osEventFlagsDelete(event_flags_);
}

bool Mpu6050::Init() {
  // Register I2C callbacks for this specific handle
  HAL_StatusTypeDef cb_status;
  cb_status = HAL_I2C_RegisterCallback(hi2c_, HAL_I2C_MEM_TX_COMPLETE_CB_ID, I2C_MemTxCpltCallback);
  if (cb_status != HAL_OK) return false;
  cb_status = HAL_I2C_RegisterCallback(hi2c_, HAL_I2C_MEM_RX_COMPLETE_CB_ID, I2C_MemRxCpltCallback);
  if (cb_status != HAL_OK) return false;
  cb_status = HAL_I2C_RegisterCallback(hi2c_, HAL_I2C_ERROR_CB_ID, I2C_ErrorCallback);
  if (cb_status != HAL_OK) return false;

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

  gx = static_cast<float>(raw_gx) / 131.0f;
  gy = static_cast<float>(raw_gy) / 131.0f;
  gz = static_cast<float>(raw_gz) / 131.0f;

  return true;
}

void Mpu6050::OnTransferComplete() {
  osEventFlagsSet(event_flags_, FLAG_DONE);
}

void Mpu6050::OnTransferError() {
  osEventFlagsSet(event_flags_, FLAG_ERROR);
}

bool Mpu6050::WaitForTransfer(uint32_t timeout_ms) {
  uint32_t flags = osEventFlagsWait(event_flags_, FLAG_DONE | FLAG_ERROR, osFlagsWaitAny, timeout_ms);
  if (flags & FLAG_ERROR) {
    std::printf("MPU6050: I2C Transfer Error reported via callback\n");
    return false;
  }
  if (flags == static_cast<uint32_t>(osErrorTimeout)) {
    std::printf("MPU6050: I2C Transfer Timeout\n");
    return false;
  }
  return (flags & FLAG_DONE) != 0;
}

bool Mpu6050::WriteReg(uint8_t reg, uint8_t value) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(hi2c_, I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1);
  if (status != HAL_OK) return false;
  return WaitForTransfer();
}

bool Mpu6050::ReadReg(uint8_t reg, uint8_t& value) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(hi2c_, I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1);
  if (status != HAL_OK) return false;
  return WaitForTransfer();
}

bool Mpu6050::ReadBytes(uint8_t reg, uint8_t* data, uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(hi2c_, I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, size);
  if (status != HAL_OK) return false;
  return WaitForTransfer();
}

}  // namespace stfoc
