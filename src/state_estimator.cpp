#include "legged_state_estimator/state_estimator.hpp"


namespace legged_state_estimator {
StateEstimator::StateEstimator(const StateEstimatorSettings& settings)
  : inekf_(settings.inekf_noise_params),
    inekf_state_(),
    inekf_leg_kinematics_(),
    robot_(settings.path_to_urdf, settings.contact_frames),
    contact_estimator_(robot_, settings.contact_estimator_settings),
    lpf_gyro_(settings.dt, settings.lpf_gyro_cutoff),
    lpf_dqJ_(settings.dt, settings.lpf_dqJ_cutoff, robot_.nJ()),
    lpf_ddqJ_(settings.dt, settings.lpf_ddqJ_cutoff, robot_.nJ()),
    lpf_tauJ_(settings.dt, settings.lpf_tauJ_cutoff, robot_.nJ()),
    dt_(settings.dt),
    imu_raw_(Vector6d::Zero()) {
  Matrix6d cov_leg;
  for (int i=0; i<4; ++i) {
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
    lpf_gyro_(),
    lpf_dqJ_(),
    lpf_ddqJ_(),
    lpf_tauJ_(),
    dt_(0) {
}


StateEstimator::~StateEstimator() {}


void StateEstimator::init(const Eigen::Vector3d& base_pos,
                          const Eigen::Vector4d& base_quat,
                          const Eigen::Vector3d& base_lin_vel,
                          const Eigen::Vector3d& imu_gyro_bias,
                          const Eigen::Vector3d& imu_lin_accel_bias) {
  inekf_state_.setPosition(base_pos);
  inekf_state_.setRotation(Eigen::Quaterniond(base_quat).toRotationMatrix());
  inekf_state_.setVelocity(base_lin_vel);
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
  imu_raw_.template head<3>() = imu_gyro_raw;
  imu_raw_.template tail<3>() = imu_lin_accel_raw;
  inekf_.Propagate(imu_raw_, dt_);
  lpf_gyro_.update(imu_gyro_raw-inekf_.getState().getGyroscopeBias());
  lpf_dqJ_.update(dqJ);
  lpf_ddqJ_.update(ddqJ);
  lpf_tauJ_.update(tauJ);
  robot_.updateLegKinematics(qJ, dqJ);
  robot_.updateLegDynamics(qJ, dqJ, ddqJ);
  contact_estimator_.update(robot_, lpf_tauJ_.getEstimate(), f_raw);
  inekf_.setContacts(contact_estimator_.getContactState());
  for (int i=0; i<robot_.numContacts(); ++i) {
    inekf_leg_kinematics_[i].setContactPosition(
        robot_.getContactPosition(i)-robot_.getBasePosition());
  }
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
  imu_raw_.template head<3>() = imu_gyro_raw;
  imu_raw_.template tail<3>() = imu_lin_accel_raw;
  inekf_.Propagate(imu_raw_, dt_);
  lpf_gyro_.update(imu_gyro_raw-inekf_.getState().getGyroscopeBias());
  lpf_dqJ_.update(dqJ);
  lpf_ddqJ_.update(ddqJ);
  lpf_tauJ_.update(tauJ);
}

} // namespace legged_state_estimator
