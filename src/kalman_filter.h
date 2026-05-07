#ifndef STFOC_KALMAN_FILTER_H
#define STFOC_KALMAN_FILTER_H

// Arduino defines a macro F() for flash strings which conflicts with Eigen
#ifdef F
#undef F
#endif
#include <Eigen/Dense>

namespace stfoc {

class KalmanFilter {
 public:
  KalmanFilter(float Q_angle = 0.001f, float Q_bias = 0.003f, float R_measure = 0.03f)
      : Q_angle_(Q_angle), Q_bias_(Q_bias), R_measure_(R_measure) {
    x_.setZero();
    P_.setZero();
  }

  // Update the Kalman filter with a new measurement
  // newAngle: measurement from accelerometer (radians)
  // newRate: measurement from gyroscope (radians/s, consistent with angle)
  // dt: time step in seconds
  float update(float newAngle, float newRate, float dt) {
    // Prediction step
    // x = F * x + B * u
    // F = [1 -dt; 0 1], B = [dt; 0], u = newRate
    rate_ = newRate - x_(1);
    x_(0) += dt * rate_;
    
    // P = F * P * F^T + Q
    Eigen::Matrix2f F;
    F << 1.0f, -dt,
         0.0f, 1.0f;
    
    Eigen::Matrix2f Q;
    Q << Q_angle_ * dt, 0.0f,
         0.0f, Q_bias_ * dt;
    
    P_ = F * P_ * F.transpose() + Q;

    // Measurement Update step
    // y = z - H * x
    // H = [1 0]
    float y = newAngle - x_(0);
    
    // S = H * P * H^T + R
    float S = P_(0, 0) + R_measure_;
    
    // K = P * H^T * S^-1
    Eigen::Vector2f K = P_.col(0) / S;
    
    // x = x + K * y
    x_ += K * y;

    
    // P = (I - K * H) * P
    // P = P - K * (H * P)
    // H = [1 0], so H * P is P.row(0)
    float p00 = P_(0, 0);
    float p01 = P_(0, 1);
    P_.col(0) -= K * p00;
    P_.col(1) -= K * p01;

    return x_(0);
  }

  float getAngle() const { return x_(0); }
  float getRate() const { return rate_; } // rate_ was unbiased rate in original
  float getBias() const { return x_(1); }

 private:
  float Q_angle_;   // Process noise variance for the angle
  float Q_bias_;    // Process noise variance for the gyro bias
  float R_measure_; // Measurement noise variance

  Eigen::Vector2f x_; // State vector [angle, bias]
  Eigen::Matrix2f P_; // Error covariance matrix
  
  float rate_;      // Unbiased rate (updated in prediction)
};

} // namespace stfoc

#endif // STFOC_KALMAN_FILTER_H
