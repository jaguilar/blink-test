#include "CppUTest/TestHarness.h"
#include "kalman_filter.h"
#include <cmath>

TEST_GROUP(KalmanFilterTest) {
  stfoc::KalmanFilter* filter;

  void setup() {
    // Q_angle, Q_bias, R_measure
    filter = new stfoc::KalmanFilter(0.001f, 0.003f, 0.03f);
  }

  void teardown() {
    delete filter;
  }
};

TEST(KalmanFilterTest, InitialState) {
  DOUBLES_EQUAL(0.0f, filter->getAngle(), 1e-6f);
  DOUBLES_EQUAL(0.0f, filter->getBias(), 1e-6f);
}

TEST(KalmanFilterTest, UpdateConvergesToConstantAngle) {
  const float dt = 0.01f; // 100Hz
  const float target_angle = 10.0f;
  const float gyro_rate = 0.0f;

  // Run for 1 second
  for (int i = 0; i < 100; ++i) {
    filter->update(target_angle, gyro_rate, dt);
  }

  // Should be close to target_angle
  DOUBLES_EQUAL(target_angle, filter->getAngle(), 0.1f);
}

TEST(KalmanFilterTest, UpdateConvergesWithBias) {
  const float dt = 0.01f;
  const float target_angle = 0.0f;
  // Gyro has a constant bias of 5 deg/s, but actual rate is 0
  const float biased_gyro_rate = 5.0f;

  // Run for 5 seconds to let bias estimate converge
  for (int i = 0; i < 500; ++i) {
    filter->update(target_angle, biased_gyro_rate, dt);
  }

  // Bias should be close to 5.0f
  DOUBLES_EQUAL(biased_gyro_rate, filter->getBias(), 0.5f);
  // Angle should still be close to 0.0f
  DOUBLES_EQUAL(target_angle, filter->getAngle(), 0.5f);
}
