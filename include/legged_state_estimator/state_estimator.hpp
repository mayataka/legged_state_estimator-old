#ifndef LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_HPP_
#define LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_HPP_

#include <string>
#include <vector>
#include <cstdio>
#include <limits>

#include "Eigen/Core"
#include "Eigen/StdVector"

#include "inekf/InEKF.h"
#include "inekf/RobotState.h"
#include "inekf/NoiseParams.h"
#include "inekf/Observations.h"

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/robot.hpp"
#include "legged_state_estimator/contact_estimator.hpp"
#include "legged_state_estimator/low_pass_filter.hpp"
#include "legged_state_estimator/state_estimator_settings.hpp"


namespace legged_state_estimator {

class StateEstimator {
public:
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix3d = Eigen::Matrix<double, 3, 3>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  StateEstimator(const StateEstimatorSettings& settings);

  StateEstimator();

  ~StateEstimator();

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(StateEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(StateEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(StateEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(StateEstimator);

  void init(const Eigen::Vector3d& base_pos,
            const Eigen::Vector4d& base_quat,
            const Eigen::Vector3d& base_lin_vel,
            const Eigen::Vector3d& imu_gyro_bias,
            const Eigen::Vector3d& imu_lin_accel_bias);

  void update(const Eigen::Vector3d& imu_gyro_raw, 
              const Eigen::Vector3d& imu_lin_accel_raw, 
              const Eigen::VectorXd& qJ, const Eigen::VectorXd& dqJ, 
              const Eigen::VectorXd& ddqJ, const Eigen::VectorXd& tauJ, 
              const std::vector<double>& f_raw);

  void predict(const Eigen::Vector3d& imu_gyro_raw, 
               const Eigen::Vector3d& imu_lin_accel_raw, 
               const Eigen::VectorXd& qJ, const Eigen::VectorXd& dqJ, 
               const Eigen::VectorXd& ddqJ, const Eigen::VectorXd& tauJ, 
               const std::vector<double>& f_raw,
               const Eigen::Vector3d& lin_vel_pred);

  const auto getBasePositionEstimate() const {
    return inekf_.getState().getPosition();
  }

  const auto getBaseRotationEstimate() const {
    return inekf_.getState().getRotation();
  }

  const auto getBaseQuaternionEstimate() const {
    return Eigen::Quaterniond(inekf_.getState().getRotation()).coeffs();
  }

  const auto getBaseLinearVelocityEstimate() const {
    return inekf_.getState().getVelocity();
  }

  const auto getIMUGyroBiasEstimate() const {
    return inekf_.getState().getGyroscopeBias();
  }

  const auto getIMULinearAccelerationBiasEstimate() const {
    return inekf_.getState().getAccelerometerBias();
  }

  const decltype(auto) getBaseAngularVelocityEstimate() const {
    return lpf_gyro_world_.getEstimate();
  }

  const decltype(auto) getJointVelocityEstimate() const {
    return lpf_dqJ_.getEstimate();
  }

  const decltype(auto) getJointAccelerationEstimate() const {
    return lpf_ddqJ_.getEstimate();
  }

  const decltype(auto) getJointTorqueEstimate() const {
    return lpf_tauJ_.getEstimate();
  }

  const decltype(auto) getContactForceEstimate() const {
    return contact_estimator_.getContactForceEstimate();
  }

  const decltype(auto) getContactProbability() const {
    return contact_estimator_.getContactProbability();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  inekf::InEKF inekf_;
  inekf::RobotState inekf_state_;
  inekf::vectorKinematics inekf_leg_kinematics_;
  Robot robot_;
  ContactEstimator contact_estimator_;
  LowPassFilter<double, Eigen::Dynamic> lpf_dqJ_, lpf_ddqJ_, lpf_tauJ_;
  LowPassFilter<double, 3> lpf_gyro_world_, lpf_gyro_accel_world_, lpf_lin_accel_world_;
  double dt_;
  Vector3d imu_gyro_raw_world_, imu_lin_accel_raw_world_, 
           gyro_world_, gyro_world_prev_, gyro_accel_world_, gyro_accel_local_, lin_accel_local_;
  Vector6d imu_raw_;
  Matrix3d R_;
};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_HPP_ 