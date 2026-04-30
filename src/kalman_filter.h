#ifndef STFOC_KALMAN_FILTER_H
#define STFOC_KALMAN_FILTER_H

namespace stfoc {

class KalmanFilter {
 public:
  KalmanFilter(float Q_angle = 0.001f, float Q_bias = 0.003f, float R_measure = 0.03f)
      : Q_angle_(Q_angle), Q_bias_(Q_bias), R_measure_(R_measure) {
    angle_ = 0.0f;
    bias_ = 0.0f;
    P_[0][0] = 0.0f;
    P_[0][1] = 0.0f;
    P_[1][0] = 0.0f;
    P_[1][1] = 0.0f;
  }

  // Update the Kalman filter with a new measurement
  // newAngle: measurement from accelerometer (radians)
  // newRate: measurement from gyroscope (radians/s or deg/s, consistent with angle)
  // dt: time step in seconds
  float update(float newAngle, float newRate, float dt) {
    // Prediction step
    rate_ = newRate - bias_;
    angle_ += dt * rate_;

    // Update error covariance matrix - Prediction
    P_[0][0] += dt * (dt * P_[1][1] - P_[0][1] - P_[1][0] + Q_angle_);
    P_[0][1] -= dt * P_[1][1];
    P_[1][0] -= dt * P_[1][1];
    P_[1][1] += Q_bias_ * dt;

    // Measurement Update step
    float S = P_[0][0] + R_measure_;
    float K[2]; // Kalman gain
    K[0] = P_[0][0] / S;
    K[1] = P_[1][0] / S;

    float y = newAngle - angle_; // Innovation
    angle_ += K[0] * y;
    bias_ += K[1] * y;

    // Update error covariance matrix - Measurement
    float P00_temp = P_[0][0];
    float P01_temp = P_[0][1];

    P_[0][0] -= K[0] * P00_temp;
    P_[0][1] -= K[0] * P01_temp;
    P_[1][0] -= K[1] * P00_temp;
    P_[1][1] -= K[1] * P01_temp;

    return angle_;
  }

  float getAngle() const { return angle_; }
  float getRate() const { return rate_; }
  float getBias() const { return bias_; }

 private:
  float Q_angle_;   // Process noise variance for the angle
  float Q_bias_;    // Process noise variance for the gyro bias
  float R_measure_; // Measurement noise variance

  float angle_;     // Estimated angle
  float bias_;      // Estimated gyro bias
  float rate_;      // Unbiased rate

  float P_[2][2];   // Error covariance matrix
};

} // namespace stfoc

#endif // STFOC_KALMAN_FILTER_H
