#include "mpu6050.h"
#include <cstdio>

namespace stfoc {

Mpu6050* g_mpu6050 = nullptr;
osEventFlagsId_t g_imu_event_flags = nullptr;

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

Mpu6050::Mpu6050(I2C_HandleTypeDef* hi2c, osEventFlagsId_t external_event_flags, uint32_t external_event_bit)
    : hi2c_(hi2c), event_flags_(osEventFlagsNew(nullptr)),
      external_event_flags_(external_event_flags), external_event_bit_(external_event_bit),
      is_background_reading_(false), ax_(0), ay_(0), az_(0), gx_(0), gy_(0), gz_(0) {
  if (hi2c_->Instance == I2C1) {
    g_mpu6050_hi2c1 = this;
    g_mpu6050 = this;
  }
}

Mpu6050::~Mpu6050() {
  // Disable interrupts on the device (blocking call for reliability in destructor)
  uint8_t disable_int = 0x00;
  HAL_I2C_Mem_Write(hi2c_, I2C_ADDR, REG_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &disable_int, 1, 100);

  if (hi2c_->Instance == I2C1 && g_mpu6050_hi2c1 == this) {
    g_mpu6050_hi2c1 = nullptr;
  }
  if (g_mpu6050 == this) {
    g_mpu6050 = nullptr;
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

  // Set SMPLRT_DIV to 4 (200Hz sample rate if DLPF is enabled)
  if (!WriteReg(REG_SMPLRT_DIV, 0x04)) {
    std::printf("MPU6050: Failed to write SMPLRT_DIV\n");
    return false;
  }

  // Set CONFIG (DLPF) to 42Hz (Internal rate = 1kHz)
  if (!WriteReg(REG_CONFIG, 0x03)) {
    std::printf("MPU6050: Failed to write CONFIG\n");
    return false;
  }

  // Enable Data Ready Interrupt
  if (!WriteReg(REG_INT_ENABLE, 0x01)) {
    std::printf("MPU6050: Failed to write INT_ENABLE\n");
    return false;
  }

  std::printf("MPU6050: Initialization successful (500Hz interrupt enabled)\n");
  return true;
}

bool Mpu6050::ReadAccel(float& ax, float& ay, float& az) {
  if (!StartReadDMA(REG_ACCEL_XOUT_H, dma_buffer_, 14)) return false;
  if (!FinishRead()) return false;
  GetAccel(ax, ay, az);
  return true;
}

bool Mpu6050::ReadGyro(float& gx, float& gy, float& gz) {
  if (!StartReadDMA(REG_ACCEL_XOUT_H, dma_buffer_, 14)) return false;
  if (!FinishRead()) return false;
  GetGyro(gx, gy, gz);
  return true;
}

void Mpu6050::HandleInterrupt() {
  if (external_event_flags_) {
    osEventFlagsSet(external_event_flags_, external_event_bit_);
  }
  // Kick off non-blocking DMA read of all sensor data (14 bytes)
  // Only start if the I2C is not already busy with another transfer.
  if (hi2c_->State == HAL_I2C_STATE_READY) {
    is_background_reading_ = true;
    if (!StartReadDMA(REG_ACCEL_XOUT_H, dma_buffer_, 14)) {
      is_background_reading_ = false;
    }
  }
}

bool Mpu6050::FinishRead() {
  if (!WaitForTransfer()) return false;
  ParseDmaBuffer();
  return true;
}

void Mpu6050::ParseDmaBuffer() {
  // Parse Accel
  int16_t raw_ax = (static_cast<int16_t>(dma_buffer_[0]) << 8) | dma_buffer_[1];
  int16_t raw_ay = (static_cast<int16_t>(dma_buffer_[2]) << 8) | dma_buffer_[3];
  int16_t raw_az = (static_cast<int16_t>(dma_buffer_[4]) << 8) | dma_buffer_[5];

  ax_ = static_cast<float>(raw_ax) / 16384.0f;
  ay_ = static_cast<float>(raw_ay) / 16384.0f;
  az_ = static_cast<float>(raw_az) / 16384.0f;

  // Skip Temperature (dma_buffer_[6..7])

  // Parse Gyro
  int16_t raw_gx = (static_cast<int16_t>(dma_buffer_[8]) << 8) | dma_buffer_[9];
  int16_t raw_gy = (static_cast<int16_t>(dma_buffer_[10]) << 8) | dma_buffer_[11];
  int16_t raw_gz = (static_cast<int16_t>(dma_buffer_[12]) << 8) | dma_buffer_[13];

  gx_ = static_cast<float>(raw_gx) / 131.0f;
  gy_ = static_cast<float>(raw_gy) / 131.0f;
  gz_ = static_cast<float>(raw_gz) / 131.0f;
}

void Mpu6050::GetAccel(float& ax, float& ay, float& az) const {
  ax = ax_;
  ay = ay_;
  az = az_;
}

void Mpu6050::GetGyro(float& gx, float& gy, float& gz) const {
  gx = gx_;
  gy = gy_;
  gz = gz_;
}

void Mpu6050::OnTransferComplete() {
  if (is_background_reading_) {
    ParseDmaBuffer();
    is_background_reading_ = false;
  }
  osEventFlagsSet(event_flags_, FLAG_DONE);
}

void Mpu6050::OnTransferError() {
  uint32_t error_code = HAL_I2C_GetError(hi2c_);
  std::printf("MPU6050: OnTransferError, ErrorCode = 0x%08lX\n", error_code);
  is_background_reading_ = false;
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
  uint32_t start = HAL_GetTick();
  while (hi2c_->State != HAL_I2C_STATE_READY) {
    if (HAL_GetTick() - start > 100) return false;
    osDelay(1);
  }
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(hi2c_, I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1);
  if (status != HAL_OK) return false;
  return WaitForTransfer();
}

bool Mpu6050::ReadReg(uint8_t reg, uint8_t& value) {
  uint32_t start = HAL_GetTick();
  while (hi2c_->State != HAL_I2C_STATE_READY) {
    if (HAL_GetTick() - start > 100) return false;
    osDelay(1);
  }
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(hi2c_, I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1);
  if (status != HAL_OK) return false;
  return WaitForTransfer();
}

bool Mpu6050::ReadBytes(uint8_t reg, uint8_t* data, uint16_t size) {
  if (!StartReadDMA(reg, data, size)) return false;
  return WaitForTransfer();
}

bool Mpu6050::StartReadDMA(uint8_t reg, uint8_t* data, uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(hi2c_, I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, size);
  return (status == HAL_OK);
}

}  // namespace stfoc

extern "C" {
void Mpu6050_HandleInterrupt() {
  if (stfoc::g_mpu6050) {
    stfoc::g_mpu6050->HandleInterrupt();
  }
}
}
