#include "legged_state_estimator/state_estimator.hpp"


namespace legged_state_estimator {

StateEstimator::StateEstimator(const StateEstimatorSettings& settings)
  : inekf_(settings.inekf_noise_params),
    inekf_state_(),
    inekf_leg_kinematics_(),
    robot_(settings.path_to_urdf, settings.imu_frame, settings.contact_frames),
    contact_estimator_(robot_, settings.contact_estimator_settings),
    lpf_gyro_accel_world_(settings.dt, settings.lpf_gyro_accel_cutoff),
    lpf_lin_accel_world_(settings.dt, settings.lpf_lin_accel_cutoff),
    lpf_dqJ_(settings.dt, settings.lpf_dqJ_cutoff, robot_.nJ()),
    lpf_ddqJ_(settings.dt, settings.lpf_ddqJ_cutoff, robot_.nJ()),
    lpf_tauJ_(settings.dt, settings.lpf_tauJ_cutoff, robot_.nJ()),
    dt_(settings.dt),
    contact_position_cov_(settings.contact_position_noise*settings.contact_position_noise), 
    contact_rotation_cov_(settings.contact_rotation_noise*settings.contact_rotation_noise),
    imu_gyro_raw_world_(Vector3d::Zero()), 
    imu_gyro_raw_world_prev_(Vector3d::Zero()), 
    imu_gyro_accel_world_(Vector3d::Zero()), 
    imu_gyro_accel_local_(Vector3d::Zero()), 
    imu_lin_accel_raw_world_(Vector3d::Zero()), 
    imu_lin_accel_local_(Vector3d::Zero()),
    imu_raw_(Vector6d::Zero()),
    R_(Matrix3d::Identity()) {
  Matrix6d cov_leg = Matrix6d::Zero();
  cov_leg.topLeftCorner<3, 3>() = contact_position_cov_ * Eigen::Matrix3d::Identity();
  cov_leg.bottomRightCorner<3, 3>() = contact_rotation_cov_ * Eigen::Matrix3d::Identity();
  for (int i=0; i<settings.contact_frames.size(); ++i) {
    inekf_leg_kinematics_.emplace_back(i, Eigen::Matrix4d::Identity(), cov_leg);
  }
  imu_raw_.setZero();
}


StateEstimator::StateEstimator() 
  : inekf_(),
    inekf_state_(),
    inekf_leg_kinematics_(),
    robot_(),
    contact_estimator_(),
    lpf_gyro_accel_world_(),
    lpf_lin_accel_world_(),
    lpf_dqJ_(),
    lpf_ddqJ_(),
    lpf_tauJ_(),
    dt_(0),
    imu_gyro_raw_world_(Vector3d::Zero()), 
    imu_gyro_raw_world_prev_(Vector3d::Zero()), 
    imu_gyro_accel_world_(Vector3d::Zero()), 
    imu_gyro_accel_local_(Vector3d::Zero()), 
    imu_lin_accel_raw_world_(Vector3d::Zero()), 
    imu_lin_accel_local_(Vector3d::Zero()),
    imu_raw_(Vector6d::Zero()),
    R_(Matrix3d::Identity()) {
}


StateEstimator::~StateEstimator() {}


void StateEstimator::init(const Eigen::Vector3d& base_pos,
                          const Eigen::Vector4d& base_quat,
                          const Eigen::Vector3d& base_lin_vel_world,
                          const Eigen::Vector3d& imu_gyro_bias,
                          const Eigen::Vector3d& imu_lin_accel_bias) {
  inekf_state_.setPosition(base_pos);
  inekf_state_.setRotation(Eigen::Quaterniond(base_quat).toRotationMatrix());
  inekf_state_.setVelocity(base_lin_vel_world);
  inekf_state_.setGyroscopeBias(imu_gyro_bias);
  inekf_state_.setAccelerometerBias(imu_lin_accel_bias);
  inekf_.setState(inekf_state_);
}


void StateEstimator::update(const Eigen::Vector3d& imu_gyro_raw, 
                            const Eigen::Vector3d& imu_lin_accel_raw, 
                            const Eigen::VectorXd& qJ, 
                            const Eigen::VectorXd& dqJ, 
                            const Eigen::VectorXd& ddqJ, 
                            const Eigen::VectorXd& tauJ, 
                            const std::vector<double>& f_raw) {
  // Process IMU measurements in InEKF
  imu_raw_.template head<3>() = imu_gyro_raw;
  imu_raw_.template tail<3>() = imu_lin_accel_raw;
  inekf_.Propagate(imu_raw_, dt_);
  // Process IMU measurements in LPFs
  imu_gyro_raw_world_.noalias() = getBaseRotationEstimate() * (imu_gyro_raw - getIMUGyroBiasEstimate());
  imu_gyro_accel_world_.noalias() = (imu_gyro_raw_world_ - imu_gyro_raw_world_prev_) / dt_;
  lpf_gyro_accel_world_.update(imu_gyro_accel_world_);
  imu_gyro_accel_local_.noalias() 
      = getBaseRotationEstimate().transpose() * lpf_gyro_accel_world_.getEstimate();
  imu_gyro_raw_world_prev_ = imu_gyro_raw_world_;
  imu_lin_accel_raw_world_.noalias() = getBaseRotationEstimate() * (imu_lin_accel_raw - getIMULinearAccelerationBiasEstimate());
  lpf_lin_accel_world_.update(imu_lin_accel_raw_world_);
  imu_lin_accel_local_.noalias()
      = getBaseRotationEstimate().transpose() * lpf_lin_accel_world_.getEstimate();
  lpf_dqJ_.update(dqJ);
  lpf_ddqJ_.update(ddqJ);
  lpf_tauJ_.update(tauJ);
  // Update contact info
  robot_.updateLegKinematics(qJ);
  robot_.updateLegDynamics(qJ, dqJ);
  contact_estimator_.update(robot_, lpf_tauJ_.getEstimate(), f_raw);
  inekf_.setContacts(contact_estimator_.getContactState());
  const double contact_force_cov = contact_estimator_.getContactForceCovariance();
  for (int i=0; i<robot_.numContacts(); ++i) {
    inekf_leg_kinematics_[i].setContactPosition(
        robot_.getContactPosition(i)-robot_.getBasePosition());
    inekf_leg_kinematics_[i].setContactPositionCovariance(
        contact_force_cov*Eigen::Matrix3d::Identity());
  }
  // Process kinematics measurements in InEKF
  inekf_.CorrectKinematics(inekf_leg_kinematics_);
}


void StateEstimator::predict(const Eigen::Vector3d& imu_gyro_raw, 
                             const Eigen::Vector3d& imu_lin_accel_raw, 
                             const Eigen::VectorXd& qJ, 
                             const Eigen::VectorXd& dqJ, 
                             const Eigen::VectorXd& ddqJ, 
                             const Eigen::VectorXd& tauJ, 
                             const std::vector<double>& f_raw,
                             const Eigen::Vector3d& lin_vel_pred) {
}

} // namespace legged_state_estimator