#ifndef LEGGED_STATE_ESTIMATOR_LEG_ODOMETRY_HPP_
#define LEGGED_STATE_ESTIMATOR_LEG_ODOMETRY_HPP_

#include <string>
#include <array>
#include <limits>

#include "Eigen/Core"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"
#include "legged_state_estimator/complementary_filter.hpp"


namespace legged_state_estimator {

template <typename Scalar>
class LegOdometry {
public:
  using Jacobian6D = types::Matrix<Scalar, 6, 18>;
  using Vector19 = types::Vector19<Scalar>;
  using Vector18 = types::Vector18<Scalar>;
  using Vector12 = types::Vector12<Scalar>;
  using Vector4  = types::Vector4<Scalar>;
  using Vector3  = types::Vector3<Scalar>;
  using Quaternion = types::Quaternion<Scalar>;

  LegOdometry(const std::string& path_to_urdf, 
              const std::array<int, 4>& contact_frames,
              const Scalar dt, const Scalar cf_cutoff_freq)
    : contact_frames_(contact_frames),
      model_(),
      data_(),
      jac_6d_(), 
      q_(Vector19::Zero()),
      v_(Vector18::Zero()),
      base_position_estimate_(Vector3::Zero()),
      base_velocity_estimate_(Vector3::Zero()),
      vec3d_tmp_(Vector3::Zero()),
      contact_frame_position_(),
      local_contact_frame_position_estimate_(),
      local_contact_frame_velocity_estimate_(),
      contact_frame_position_estimate_(),
      contact_frame_velocity_estimate_(),
      cf_contact_frame_position_(),
      dt_(dt) {
    pinocchio::Model dmodel;
    pinocchio::urdf::buildModel(path_to_urdf, 
                                pinocchio::JointModelFreeFlyer(), dmodel);
    model_ = dmodel.cast<Scalar>();
    data_ = pinocchio::DataTpl<Scalar>(model_);
    jac_6d_.fill(Jacobian6D::Zero());
    contact_frame_position_.fill(Vector3::Zero());
    local_contact_frame_position_estimate_.fill(Vector3::Zero());
    local_contact_frame_velocity_estimate_.fill(Vector3::Zero());
    contact_frame_position_estimate_.fill(Vector3::Zero());
    contact_frame_velocity_estimate_.fill(Vector3::Zero());
    cf_contact_frame_position_.fill(ComplementaryFilter<Scalar, 3>(dt, cf_cutoff_freq));
  }

  LegOdometry()
    : contact_frames_(),
      model_(),
      data_(),
      jac_6d_(), 
      q_(Vector19::Zero()),
      v_(Vector18::Zero()),
      base_position_estimate_(Vector3::Zero()),
      base_velocity_estimate_(Vector3::Zero()),
      vec3d_tmp_(Vector3::Zero()),
      contact_frame_position_(),
      local_contact_frame_position_estimate_(),
      local_contact_frame_velocity_estimate_(),
      contact_frame_position_estimate_(),
      contact_frame_velocity_estimate_(),
      cf_contact_frame_position_(),
      dt_(0) {
    jac_6d_.fill(Jacobian6D::Zero());
    contact_frame_position_.fill(Vector3::Zero());
    local_contact_frame_position_estimate_.fill(Vector3::Zero());
    local_contact_frame_velocity_estimate_.fill(Vector3::Zero());
    contact_frame_position_estimate_.fill(Vector3::Zero());
    contact_frame_velocity_estimate_.fill(Vector3::Zero());
  }

  ~LegOdometry() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(LegOdometry);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(LegOdometry);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(LegOdometry);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(LegOdometry);

  void updateBaseStateEstimation(const Quaternion& quat, const Vector3& gyro, 
                                 const Vector12& qJ, const Vector12& dqJ,
                                 const Vector4& contact_probability) {
    q_.template head<3>().setZero();
    q_.coeffRef(3) = quat.x();
    q_.coeffRef(4) = quat.y();
    q_.coeffRef(5) = quat.z();
    q_.coeffRef(6) = quat.w();
    q_.template tail<12>() = qJ;
    v_.template head<3>().setZero();
    v_.template segment<3>(3) = gyro;
    v_.template tail<12>() = dqJ;
    pinocchio::normalize(model_, q_);
    pinocchio::framesForwardKinematics(model_, data_, q_);
    pinocchio::computeJointJacobians(model_, data_, q_);
    const double contact_probability_sum = contact_probability.sum();
    constexpr double eps = std::numeric_limits<Scalar>::epsilon();
    if (contact_probability_sum > eps) {
      contact_probability_normalized_ = contact_probability / contact_probability_sum;
    }
    else {
      contact_probability_normalized_.fill(0.25);
    }
    base_velocity_estimate_.setZero();
    base_position_estimate_.setZero();
    for (int i=0; i<4; ++i) {
      pinocchio::getFrameJacobian(model_, data_, contact_frames_[i], 
                                  pinocchio::LOCAL_WORLD_ALIGNED, jac_6d_[i]);
      local_contact_frame_position_estimate_[i]
          = data_.oMf[contact_frames_[i]].translation()
              - data_.oMf[base_frame_].translation();
      local_contact_frame_velocity_estimate_[i].noalias()
          = jac_6d_[i].template topRows<3>() * v_; 
      const Scalar weight = contact_probability_normalized_.coeff(i);
      base_velocity_estimate_.noalias() 
          -= weight * local_contact_frame_velocity_estimate_[i];
      base_position_estimate_.noalias() 
          += weight * (- local_contact_frame_position_estimate_[i]
                        + contact_frame_position_estimate_[i]);
    }
  }

  void updateContactPositionEstimation(const Vector3& base_lin_vel_est,
                                       const Vector4& contact_probability) {
    for (int i=0; i<4; ++i) {
      contact_frame_velocity_estimate_[i] = local_contact_frame_velocity_estimate_[i] 
                                              + base_lin_vel_est;
      const Scalar non_contact_prob = 1.0 - contact_probability.coeff(i);
      const Vector3 current_contact_frame_pos = cf_contact_frame_position_[i].getEstimate();
      cf_contact_frame_position_[i].update(contact_frame_velocity_estimate_[i], 
                                           current_contact_frame_pos, 
                                           (1.0-non_contact_prob));
      contact_frame_position_estimate_[i] = cf_contact_frame_position_[i].getEstimate();
    }
  }

  void resetBaseStateEstimation(const Scalar base_x=0, const Scalar base_y=0,
                                const Vector4& contact_height=Vector4::Zero()) {
    Scalar base_z = 0;
    for (int i=0; i<4; ++i) {
      constexpr Scalar weight = 0.25; // weight due to the sensed contact force is fixed 
      base_z += weight * (- local_contact_frame_position_estimate_[i].coeff(2) 
                            + contact_height.coeff(i));
    }
    base_position_estimate_ << base_x, base_y, base_z;
  }

  void resetContactPositionEstimation(const Scalar base_x=0, const Scalar base_y=0,
                                      const Vector4& contact_height=Vector4::Zero()) {
    for (int i=0; i<4; ++i) {
      contact_frame_position_estimate_[i] 
          << base_x + local_contact_frame_position_estimate_[i].coeff(0),
             base_y + local_contact_frame_position_estimate_[i].coeff(1),
             contact_height.coeff(i);
      cf_contact_frame_position_[i].reset(contact_frame_position_estimate_[i]);
    }
  }

  const Vector3& getBaseLinearVelocityEstimate() const {
    return base_velocity_estimate_;
  }

  const Vector3& getBasePositionEstimate() const {
    return base_position_estimate_;
  }

  const std::array<Vector3, 4>& getContactFramePositionEstimate() const {
    return contact_frame_position_estimate_;
  }

  const Vector3& getContactFramePositionEstimate(const int contact_id) const {
    return contact_frame_position_estimate_[contact_id];
  }

  const std::array<Vector3, 4>& getContactFrameVelocityEstimate() const {
    return contact_frame_velocity_estimate_;
  }

  const Vector3& getContactFrameVelocityEstimate(const int contact_id) const {
    return contact_frame_velocity_estimate_[contact_id];
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  static constexpr int base_frame_ = 1;
  std::array<int, 4> contact_frames_;
  pinocchio::ModelTpl<Scalar> model_;
  pinocchio::DataTpl<Scalar> data_;
  std::array<Jacobian6D, 4> jac_6d_;
  Vector19 q_;
  Vector18 v_;
  Vector4 contact_probability_normalized_;
  Vector3 base_position_estimate_, base_velocity_estimate_, vec3d_tmp_;
  std::array<Vector3, 4> contact_frame_position_,
                         local_contact_frame_position_estimate_,
                         local_contact_frame_velocity_estimate_,
                         contact_frame_position_estimate_,
                         contact_frame_velocity_estimate_; 
  std::array<ComplementaryFilter<Scalar, 3>, 4> cf_contact_frame_position_;
  Scalar dt_;
  //  = 1;
  // int FL_foot_ = 14;
  // int FR_foot_ = 24;
  // int RL_foot_ = 34;
  // int RR_foot_ = 44;
  //  = {FL_foot_, FR_foot_, RL_foot_, RR_foot_};
};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_LEG_ODOMETRY_HPP_ 