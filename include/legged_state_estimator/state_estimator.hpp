#ifndef LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_HPP_
#define LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_HPP_

#include <string>
#include <vector>
#include <array>
#include <cstdio>
#include <limits>

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"
#include "legged_state_estimator/robot.hpp"
#include "legged_state_estimator/extended_kalman_filter.hpp"
#include "legged_state_estimator/contact_estimator.hpp"
#include "legged_state_estimator/low_pass_filter.hpp"
#include "legged_state_estimator/state_estimator_settings.hpp"


namespace legged_state_estimator {

template <typename Scalar>
class StateEstimator {
public:
  using Matrix3  = types::Matrix3<Scalar>;
  using Vector12 = types::Vector12<Scalar>;
  using Vector4  = types::Vector4<Scalar>;
  using Vector3  = types::Vector3<Scalar>;
  using Quaternion = types::Quaternion<Scalar>;

  StateEstimator(const StateEstimatorSettings<Scalar>& settings)
    : robot_(settings.path_to_urdf, settings.contact_frames),
      ekf_(),
      contact_estimator_(settings.dt, settings.force_compl_cutoff_freq, 
                         settings.beta1_logistic_reg, 
                         settings.beta0_logistic_reg, 
                         settings.force_sensor_bias),
      lpf_gyro_(settings.dt, settings.lpf_gyro_cutoff),
      lpf_dqJ_(settings.dt, settings.lpf_dqJ_cutoff),
      lpf_tauJ_(settings.dt, settings.lpf_tauJ_cutoff),
      R_(Matrix3::Identity()),
      base_linear_acc_nobias_skew_(Matrix3::Zero()),
      base_angular_vel_nobias_(Vector3::Zero()), 
      base_linear_acc_nobias_(Vector3::Zero()),
      gravity_accel_(Vector3::Zero()),
      dt_(settings.dt) {
    gravity_accel_ << 0, 0, -9.81;
  }

  StateEstimator() 
    : robot_(),
      ekf_(),
      contact_estimator_(),
      lpf_gyro_(),
      lpf_dqJ_(),
      lpf_tauJ_(),
      R_(Matrix3::Identity()),
      base_linear_acc_nobias_skew_(Matrix3::Zero()),
      base_angular_vel_nobias_(Vector3::Zero()), 
      base_linear_acc_nobias_(Vector3::Zero()),
      gravity_accel_(Vector3::Zero()),
      dt_(0) {
  }

  ~StateEstimator() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(StateEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(StateEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(StateEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(StateEstimator);

  void update(const Vector3& imu_gyro_raw, const Vector3& imu_lin_accel_raw, 
              const Vector12& qJ, const Vector12& dqJ, const Vector12& tauJ, 
              const Vector4& f_raw) {
    setupEKFPrediction(imu_gyro_raw, imu_lin_accel_raw);
    ekf_.predictionStep();
    lpf_gyro_.update(imu_gyro_raw-getIMUGyroBiasEstimate());
    lpf_dqJ_.update(dqJ);
    lpf_tauJ_.update(tauJ);
    setupEKFMeasurement(qJ, dqJ, lpf_tauJ_.getEstimate(), f_raw);
    ekf_.filteringStep();
    ekf_.x_hat.template head<7>() 
        = robot_.integrateBaseConfiguration(ekf_.x_pred.template head<3>(),
                                            ekf_.x_pred.template segment<4>(3),
                                            ekf_.dx_hat.template head<3>(3),
                                            ekf_.dx_hat.template segment<3>(3));
    ekf_.x_hat.template tail<9>()
        = ekf_.x_pred.template tail<9>() + ekf_.dx_hat.template tail<9>();
  }

  void predict(const Vector3& imu_gyro_raw, const Vector3& imu_lin_accel_raw, 
               const Vector12& qJ, const Vector12& dqJ, const Vector12& tauJ, 
               const Vector4& f_raw, const Vector3& lin_vel_pred) {
    setupEKFPrediction(imu_gyro_raw, imu_lin_accel_raw);
    ekf_.predictionStep();
    lpf_gyro_.update(imu_gyro_raw-getIMUGyroBiasEstimate());
    lpf_dqJ_.update(dqJ);
    lpf_tauJ_.update(tauJ);
    ekf_.y = lin_vel_pred;
    ekf_.filteringStep();
    ekf_.x_hat.template head<7>() 
        = robot_.integrateBaseConfiguration(ekf_.x_pred.template head<3>(),
                                            ekf_.x_pred.template segment<4>(3),
                                            ekf_.dx_hat.template head<3>(3),
                                            ekf_.dx_hat.template segment<3>(3));
    ekf_.x_hat.template tail<9>()
        = ekf_.x_pred.template tail<9>() + ekf_.dx_hat.template tail<9>();
  }

  const Vector3& getBasePositionEstimate() const {
    return ekf_.x_hat.template segment<3>(0);
  }

  const Vector4& getBaseOrientationEstimate() const {
    return ekf_.x_hat.template segment<4>(3);
  }

  const Vector3& getBaseLinearVelocityEstimate() const {
    return ekf_.x_hat.template segment<3>(7);
  }

  const Vector3& getIMUGyroBiasEstimate() const {
    return ekf_.x_hat.template segment<3>(10);
  }

  const Vector3& getIMULinearAccelerationBiasEstimate() const {
    return ekf_.x_hat.template segment<3>(13);
  }

  const Vector3& getBaseAngularVelocityEstimate() const {
    return lpf_gyro_.getEstimate();
  }

  const Vector12& getJointVelocityEstimate() const {
    return lpf_dqJ_.getEstimate();
  }

  const Vector12& getJointTorqueEstimate() const {
    return lpf_tauJ_.getEstimate();
  }

  Scalar getContactProbability(const int contact_id) const {
    return contact_estimator_.getContactProbability(contact_id);
  }

  const Vector4& getContactProbability() const {
    return contact_estimator_.getContactProbability();
  }

  Scalar getNonContactProbability() const {
    return contact_estimator_.getNonContactProbability();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Robot<Scalar> robot_;
  static constexpr int ekf_dimx  = 16;
  static constexpr int ekf_dimdx = 15;
  static constexpr int ekf_dimy  = 3;
  ExtendedKalmanFilter<Scalar, ekf_dimx, ekf_dimdx, ekf_dimy> ekf_;
  ContactEstimator<Scalar> contact_estimator_;
  LowPassFilter<Scalar, 12> lpf_dqJ_, lpf_tauJ_;
  LowPassFilter<Scalar, 3> lpf_gyro_;
  Matrix3 R_, base_linear_acc_nobias_skew_;
  Vector3 base_angular_vel_nobias_, base_linear_acc_nobias_, gravity_accel_;
  Scalar dt_;

  void setupEKFPrediction(const Vector3& imu_gyro_raw, 
                          const Vector3& imu_linear_acc_raw) {
    // process imu info (angular vel and linear accel)
    const auto& quat = ekf_.x_hat.segment<4>(3);
    R_ = Quaternion(quat.coeff(3), quat.coeff(0), quat.coeff(1), quat.coeff(2)).toRotationMatrix();
    base_angular_vel_nobias_.noalias() = imu_gyro_raw - getIMUGyroBiasEstimate();
    base_linear_acc_nobias_.noalias() 
        = R_ * (imu_linear_acc_raw - getIMULinearAccelerationBiasEstimate()) + gravity_accel_;
    pinocchio::skew(base_linear_acc_nobias_, base_linear_acc_nobias_skew_);
    // updates the base kinematics (SE3 prediction and its Jacobians)
    robot_.updateBaseKinematics(getBasePositionEstimate(), 
                                getBaseOrientationEstimate(), 
                                getBaseLinearVelocityEstimate(), 
                                base_angular_vel_nobias_, 
                                base_linear_acc_nobias_, dt_);
    // EKF prediction
    ekf_.x_pred.template head<3>()     = robot_.getBasePosition();
    ekf_.x_pred.template segment<4>(3) = robot_.getBaseOrientation();
    ekf_.x_pred.template segment<3>(7) = robot_.getBaseLinearVelocity();
    ekf_.x_pred.template tail<6>()     = ekf_.x_hat.template tail<6>();
    ekf_.y_pred = ekf_.f.template segment<3>(7);
    ekf_.A.template block<6, 6>(0, 0) = robot_.getBaseJacobianWrtConfiguration();
    ekf_.A.template block<6, 3>(0, 6) = dt_ * robot_.getBaseJacobianWrtVelocity().template leftCols<3>();
    ekf_.A.template block<6, 3>(0, 9) = - dt_ * robot_.getBaseJacobianWrtVelocity().template rightCols<3>();
    ekf_.A.template block<3, 3>(6, 3).noalias() = - dt_ * R_ * base_linear_acc_nobias_skew_;
    ekf_.A.template block<3, 3>(6, 6).setIdentity();
    ekf_.A.template block<3, 3>(6, 9) = - dt_ * R_;
    ekf_.C = ekf_.A.template middleRows<3>(6);
    // ekf_.G.template block<>() = ;
  }

  void setupEKFMeasurement(const Vector12& qJ, const Vector12& dqJ, 
                           const Vector12& tauJ, const Vector4& f_raw) {
    robot_.updateLocalKinematics(base_angular_vel_nobias_, qJ, dqJ);
    robot_.updateLocalDynamics(qJ, dqJ);
    contact_estimator_.update(robot_, tauJ, f_raw);
    ekf_.y.setZero();
    const double contact_prob_sum 
        = contact_estimator_.getContactProbability().template lpNorm<2>();
    constexpr auto eps = std::sqrt(std::numeric_limits<Scalar>::epsilon());
    // EKF measurements (filtering step)
    if (contact_prob_sum > eps) {
      for (int i=0; i<4; ++i) {
        ekf_.y.noalias() 
            -= (contact_estimator_.getContactProbability(i)/contact_prob_sum) 
                * robot_.getContatVelocity(i);
      }
    }
  }
};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_HPP_ 