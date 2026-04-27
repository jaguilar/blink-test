#include "CppUTest/TestHarness.h"
#include "mpu6050.h"
#include <cstdio>

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
