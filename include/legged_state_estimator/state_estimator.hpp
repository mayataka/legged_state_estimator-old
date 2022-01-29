#ifndef LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_HPP_
#define LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_HPP_

#include <string>
#include <vector>
#include <array>
#include <cstdio>

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"
#include "legged_state_estimator/imu_bias_lock.hpp"
#include "legged_state_estimator/contact_estimator.hpp"
#include "legged_state_estimator/leg_odometry.hpp"
#include "legged_state_estimator/low_pass_filter.hpp"
#include "legged_state_estimator/high_pass_filter.hpp"
#include "legged_state_estimator/complementary_filter.hpp"
#include "legged_state_estimator/state_estimator_settings.hpp"


namespace legged_state_estimator {

template <typename Scalar>
class StateEstimator {
public:
  using Matrix6  = types::Matrix6<Scalar>;
  using Matrix4  = types::Matrix4<Scalar>;
  using Matrix3  = types::Matrix3<Scalar>;
  using Vector12 = types::Vector12<Scalar>;
  using Vector6  = types::Vector6<Scalar>;
  using Vector4  = types::Vector4<Scalar>;
  using Vector3  = types::Vector3<Scalar>;
  using Quaternion = types::Quaternion<Scalar>;

  StateEstimator(const StateEstimatorSettings<Scalar>& settings)
    : imu_bias_lock_(),
      contact_estimator_(settings.force_sensor_bias, 
                         settings.contact_window_filter_size, 
                         settings.contact_probability_beta1, 
                         settings.contact_probability_beta0),
      leg_odometry(settings.path_to_urdf, settings.contact_frames, 
                   settings.dt, settings.hpf_contact_frame_pos_cutoff),
      R_(Matrix3::Identity()),
      gravity_accel_(Vector3::Zero()), 
      accel_(Vector3::Zero()),
      base_lin_vel_tmp_(Vector3::Zero()), 
      base_pos_tmp_(Vector3::Zero()),
      lpf_dqJ_(settings.dt, settings.lpf_dqJ_cutoff),
      lpf_gyro_(settings.dt, settings.lpf_gyro_cutoff),
      cf_base_lin_vel_(settings.dt, settings.cf_base_lin_vel_cutoff),
      cf_base_pos_(settings.dt, settings.cf_base_pos_cutoff),
      dt_(settings.dt) {
    gravity_accel_ << 0, 0, -9.81;
  }

  StateEstimator() 
    : imu_bias_lock_(),
      contact_estimator_(),
      leg_odometry(),
      R_(Matrix3::Identity()),
      gravity_accel_(Vector3::Zero()), 
      accel_(Vector3::Zero()),
      base_lin_vel_tmp_(Vector3::Zero()), 
      base_pos_tmp_(Vector3::Zero()),
      lpf_dqJ_(),
      lpf_gyro_(),
      cf_base_lin_vel_(),
      cf_base_pos_(),
      dt_(0) {
  }

  ~StateEstimator() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(StateEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(StateEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(StateEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(StateEstimator);

  void resetCalibration() {
    imu_bias_lock_.reset();
    contact_estimator_.reset();
    lpf_dqJ_.reset();
    lpf_gyro_.reset();
    cf_base_lin_vel_.reset();
    cf_base_pos_.reset();
  }

  void addCalibrationData(const Vector3& gyro_raw, 
                          const Vector3& accel_raw) {
    imu_bias_lock_.addCalibrationData(gyro_raw, accel_raw);
  }

  void doCalibration(const Quaternion& quat) {
    imu_bias_lock_.doCalibration(quat);
  }

  void update(const Quaternion& quat, const Vector3& imu_gyro_raw, 
              const Vector3& imu_lin_accel_raw, const Vector12& qJ, 
              const Vector12& dqJ, const Vector4& f_raw) {
    // process imu info (angular vel and linear accel)
    lpf_gyro_.update(imu_gyro_raw-imu_bias_lock_.getGyroBias());
    R_ = quat.toRotationMatrix();
    accel_.noalias() = R_ * (imu_lin_accel_raw-imu_bias_lock_.getAccelBias()) 
                        + gravity_accel_;
    // process contact and encoder info
    contact_estimator_.update(f_raw);
    leg_odometry.updateBaseStateEstimation(quat, lpf_gyro_.getEstimate(), qJ, dqJ, 
                                           contact_estimator_.getContactProbability());
    lpf_dqJ_.update(dqJ);
    // complementary filter 
    const Scalar non_contact_prob = contact_estimator_.getNonContactProbability();
    cf_base_lin_vel_.update(accel_, leg_odometry.getBaseLinearVelocityEstimate(), 
                            (1.0-non_contact_prob));
    cf_base_pos_.update(cf_base_lin_vel_.getEstimate(), 
                        leg_odometry.getBasePositionEstimate(), 
                        (1.0-non_contact_prob));
    // update internal contact position estimates
    leg_odometry.updateContactPositionEstimation(
        cf_base_lin_vel_.getEstimate(), contact_estimator_.getContactProbability());
  }

  // this is for when the robot is flying (ContactState::Flying)
  void predict(const Quaternion& quat, const Vector3& imu_gyro_raw,
               const Vector3& base_accel_pred, 
               const Vector12& qJ, const Vector12& dqJ) {
    // process imu info (angular vel and linear accel)
    lpf_gyro_.update(imu_gyro_raw-imu_bias_lock_.getGyroBias());
    lpf_dqJ_.update(dqJ);
    // This accel is a tmp vector.
    base_lin_vel_tmp_ = cf_base_lin_vel_.getEstimate() + dt_ * base_accel_pred;
    base_pos_tmp_ = cf_base_pos_.getEstimate() + dt_ * cf_base_lin_vel_.getEstimate();
    cf_base_lin_vel_.reset(base_lin_vel_tmp_);
    cf_base_pos_.reset(base_pos_tmp_);
  }

  void reset(const Scalar base_x=0, const Scalar base_y=0, 
             const Vector4& contact_height=Vector4::Zero()) {
    leg_odometry.resetBaseStateEstimation(base_x, base_y, contact_height);
    leg_odometry.resetContactPositionEstimation(base_x, base_y, contact_height);
    cf_base_pos_.reset(leg_odometry.getBasePositionEstimate());
  }


  const Vector3& getBaseLinearVelocityEstimate() const {
    return cf_base_lin_vel_.getEstimate();
  }

  const Vector3& getBaseAngularVelocityEstimate() const {
    return lpf_gyro_.getEstimate();
  }

  const Vector3& getBasePositionEstimate() const {
    return cf_base_pos_.getEstimate();
  }

  const Vector12& getJointVelocityEstimate() const {
    return lpf_dqJ_.getEstimate();
  }

  const Vector4& getContactForceEstimate() const {
    return contact_estimator_.getContactForceEstimate();
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

  const Vector3& getContactFramePositionEstimate(const int contact_id) const {
    return leg_odometry.getContactFramePositionEstimate(contact_id);
  }

  const std::array<Vector3, 4>& getContactFramePositionEstimate() const {
    return leg_odometry.getContactFramePositionEstimate();
  }

  const Vector3& getContactFrameVelocityEstimate(const int contact_id) const {
    return leg_odometry.getContactFrameVelocityEstimate(contact_id);
  }

  const std::array<Vector3, 4>& getContactFrameVelocityEstimate() const {
    return leg_odometry.getContactFrameVelocityEstimate();
  }


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  ImuBiasLock<Scalar> imu_bias_lock_;
  ContactEstimator<Scalar> contact_estimator_;
  LegOdometry<Scalar> leg_odometry;
  Matrix3 R_;
  Vector3 gravity_accel_, accel_, base_lin_vel_tmp_, base_pos_tmp_;
  LowPassFilter<Scalar, 12> lpf_dqJ_;
  LowPassFilter<Scalar, 3> lpf_gyro_;
  ComplementaryFilter<Scalar, 3> cf_base_lin_vel_, cf_base_pos_;
  Scalar dt_;

};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_HPP_ 