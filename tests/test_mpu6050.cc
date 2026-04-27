#include "CppUTest/TestHarness.h"
#include "mpu6050.h"
#include <cstdio>
#include <cmath>

extern I2C_HandleTypeDef hi2c1;

TEST_GROUP(Mpu6050Test) {
  stfoc::Mpu6050* imu;

  void setup() {
    imu = new stfoc::Mpu6050(&hi2c1);
  }

  void teardown() {
    delete imu;
  }
};

TEST(Mpu6050Test, InitializationAndValidation) {
  printf("\n--- MPU6050 Validation ---\n");
  bool success = imu->Init();
  
  if (success) {
    printf("MPU6050 OK\n");
    
    float ax, ay, az;
    if (imu->ReadAccel(ax, ay, az)) {
      printf("Accel: X=%.3f, Y=%.3f, Z=%.3f (g)\n", ax, ay, az);
    }
    
    float gx, gy, gz;
    if (imu->ReadGyro(gx, gy, gz)) {
      printf("Gyro: X=%.3f, Y=%.3f, Z=%.3f (deg/s)\n", gx, gy, gz);
    }
  } else {
    printf("MPU6050 FAILED\n");
  }
  
  CHECK(success);
}

TEST(Mpu6050Test, InterruptAndNonBlockingRead) {
  printf("\n--- MPU6050 Interrupt & Non-Blocking Read Test ---\n");
  
  osEventFlagsId_t test_flags = osEventFlagsNew(nullptr);
  stfoc::Mpu6050 interrupt_imu(&hi2c1, test_flags, 0x01);
  
  bool success = interrupt_imu.Init();
  CHECK(success);
  
  printf("Waiting for interrupts (500Hz)...\n");
  
  for (int i = 0; i < 10; ++i) {
    uint32_t flags = osEventFlagsWait(test_flags, 0x01, osFlagsWaitAny, 100);
    if (flags == static_cast<uint32_t>(osErrorTimeout)) {
      FAIL("Timeout waiting for IMU interrupt");
    }
    
    // Interrupt fired and DMA read started. Now wait for it to finish.
    CHECK(interrupt_imu.FinishRead());
    
    float ax, ay, az, gx, gy, gz;
    interrupt_imu.GetAccel(ax, ay, az);
    interrupt_imu.GetGyro(gx, gy, gz);
    
    printf("[%d] Accel: [%.3f, %.3f, %.3f]g Gyro: [%.1f, %.1f, %.1f]d/s\n", 
           i, ax, ay, az, gx, gy, gz);
           
    // Basic sanity check: total acceleration should be near 1g if stationary
    float total_accel = sqrtf(ax*ax + ay*ay + az*az);
    DOUBLES_EQUAL(1.0f, total_accel, 0.2f);
  }
  
  osEventFlagsDelete(test_flags);
}
