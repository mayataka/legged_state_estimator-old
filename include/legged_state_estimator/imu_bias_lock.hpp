#ifndef LEGGED_STATE_ESTIMATOR_IMU_BIAS_LOCK_HPP_
#define LEGGED_STATE_ESTIMATOR_IMU_BIAS_LOCK_HPP_

#include <vector>
#include <cstdio>

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

template <typename Scalar>
class ImuBiasLock {
public:
  using Vector3 = types::Vector3<Scalar>;
  using Matrix3 = types::Matrix3<Scalar>;
  using Quaternion = types::Quaternion<Scalar>;

  ImuBiasLock(const Scalar gravity_acceleration=-9.81)
    // : gyro_raw_(),
    //   accel_raw_(),
    : gyro_sum_(Vector3::Zero()),
      gyro_bias_(Vector3::Zero()),
      accel_sum_(Vector3::Zero()),
      accel_bias_(Vector3::Zero()),
      gravity_accel_(Vector3::Zero()),
      // gyro_cov_(Matrix3::Zero()),
      // accel_cov_(Matrix3::Zero()), 
      size_(0) {
    gravity_accel_ << 0, 0, gravity_acceleration;
  }

  ~ImuBiasLock() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(ImuBiasLock);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(ImuBiasLock);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(ImuBiasLock);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(ImuBiasLock);

  void reset() {
    // gyro_raw_.clear();
    // accel_raw_.clear();
    gyro_sum_.setZero();
    gyro_bias_.setZero();
    accel_sum_.setZero();
    accel_bias_.setZero();
    // gyro_cov_.setZero();
    // accel_cov_.setZero(); 
    size_ = 0;
  }

  void addCalibrationData(const Vector3& gyro_raw, const Vector3& accel_raw) {
    // accel_raw_.push_back(accel_raw);
    // gyro_raw_.push_back(gyro_raw);
    accel_sum_.noalias() += accel_raw;
    gyro_sum_.noalias() += gyro_raw;
    ++size_;
  }

  void doCalibration(const Quaternion& quat) {
    if (size_ >= 1) {
      gyro_bias_ = gyro_sum_ / static_cast<Scalar>(size_);
      accel_bias_ = accel_sum_ / static_cast<Scalar>(size_);
    }
    // if (size_ >= 2) {
    //   Matrix3 gyro_cov_sum = Matrix3::Zero();
    //   for (const auto& e : gyro_raw_) {
    //     gyro_cov_sum.noalias() 
    //         += (e - gyro_bias_) * (e - gyro_bias_).transpose();  
    //   }
    //   gyro_cov_ = gyro_cov_sum / static_cast<Scalar>(size_-1);
    //   Matrix3 accel_cov_sum = Matrix3::Zero();
    //   for (const auto& e : accel_raw_) {
    //     accel_cov_sum.noalias() 
    //         += (e - accel_bias_) * (e - accel_bias_).transpose();  
    //   }
    //   accel_cov_ = accel_cov_sum / static_cast<Scalar>(size_-1);
    // }
    if (size_ >= 1) {
      const Matrix3 R = quat.toRotationMatrix();
      accel_bias_.noalias() += R.transpose() * gravity_accel_;
    }
  }

  const Vector3& getGyroBias() const {
    return gyro_bias_;
  }

  const Vector3& getAccelBias() const {
    return accel_bias_;
  }

  // const Matrix3& gyroCovariance() const {
  //   return gyro_cov_;
  // }

  // const Matrix3& accelCovariance() const {
  //   return accel_cov_;
  // }

  void print() const {
    printf("gyro_bias: %lf, %lf, %lf\n", 
            gyro_bias_.coeff(0), gyro_bias_.coeff(1), gyro_bias_.coeff(2));
    // printf("gyro_covariance: [%lf, %lf, %lf], [%lf, %lf, %lf], [%lf, %lf, %lf]\n", 
    //         gyro_cov_.coeff(0, 0), gyro_cov_.coeff(0, 1), gyro_cov_.coeff(0, 2), 
    //         gyro_cov_.coeff(1, 0), gyro_cov_.coeff(1, 1), gyro_cov_.coeff(1, 2), 
    //         gyro_cov_.coeff(2, 0), gyro_cov_.coeff(2, 1), gyro_cov_.coeff(2, 2));
    printf("accel_bias: %lf, %lf, %lf\n", 
            accel_bias_.coeff(0), accel_bias_.coeff(1), accel_bias_.coeff(2));
    // printf("accel_covariance: [%lf, %lf, %lf], [%lf, %lf, %lf], [%lf, %lf, %lf]\n", 
    //         accel_cov_.coeff(0, 0), accel_cov_.coeff(0, 1), accel_cov_.coeff(0, 2), 
    //         accel_cov_.coeff(1, 0), accel_cov_.coeff(1, 1), accel_cov_.coeff(1, 2), 
    //         accel_cov_.coeff(2, 0), accel_cov_.coeff(2, 1), accel_cov_.coeff(2, 2));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // std::vector<Vector3> gyro_raw_, accel_raw_;
  Vector3 gyro_sum_, gyro_bias_, gravity_accel_, 
          accel_sum_, accel_bias_;
  // Matrix3 gyro_cov_, accel_cov_;
  int size_;

};

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_IMU_BIAS_LOCK_HPP_ 