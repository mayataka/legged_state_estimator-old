#ifndef LEGGED_STATE_ESTIMATOR_ROBOT_HPP_
#define LEGGED_STATE_ESTIMATOR_ROBOT_HPP_

#include <string>
#include <vector>
#include <array>

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"

#include "Eigen/Core"
#include "Eigen/StdVector"
#include "Eigen/Geometry"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"


namespace legged_state_estimator {

template <typename Scalar>
class Robot {
public:
  using Jacobian6D = types::Matrix<Scalar, 6, 18>;
  using Vector19   = types::Vector19<Scalar>;
  using Vector18   = types::Vector18<Scalar>;
  using Vector12   = types::Vector12<Scalar>;
  using Vector4    = types::Vector4<Scalar>;
  using Vector3    = types::Vector3<Scalar>;
  using Quaternion = types::Quaternion<Scalar>;

  Robot(const std::string& path_to_urdf, const std::array<int, 4>& contact_frames) 
    : contact_frames_(contact_frames),
      model_(),
      data_(),
      jac_integ_dq_(Jacobian6D::Zero()),
      jac_integ_dv_(Jacobian6D::Zero()),
      q_(Vector19::Zero()),
      q_integ_(Vector19::Zero()),
      q_integ_out_(Vector19::Zero()),
      v_(Vector18::Zero()),
      a_(Vector18::Zero()),
      tau_(Vector18::Zero()),
      v_integ_(Vector18::Zero()),
      v_base_linear_integ_out_(Vector3::Zero()),
      jac_6d_(contact_frames.size(), Jacobian6D::Zero()), 
      contact_frame_velocity_(contact_frames.size(), Vector3::Zero()) {
    pinocchio::Model dmodel;
    pinocchio::urdf::buildModel(path_to_urdf, 
                                pinocchio::JointModelFreeFlyer(), dmodel);
    model_ = dmodel.cast<Scalar>();
    data_ = pinocchio::DataTpl<Scalar>(model_);
  }

  Robot() 
    : contact_frames_(),
      model_(),
      data_(),
      jac_integ_dq_(Jacobian6D::Zero()),
      jac_integ_dv_(Jacobian6D::Zero()),
      q_(Vector19::Zero()),
      q_integ_(Vector19::Zero()),
      q_integ_out_(Vector19::Zero()),
      v_(Vector18::Zero()),
      a_(Vector18::Zero()),
      tau_(Vector18::Zero()),
      v_integ_(Vector18::Zero()),
      v_base_linear_integ_out_(Vector3::Zero()),
      jac_6d_(), 
      contact_frame_velocity_() {
  }

  ~Robot() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(Robot);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(Robot);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(Robot);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(Robot);

  void updateBaseConfiguration(const Vector3& base_pos, const Vector4& base_quat,
                               const Vector3& base_linear_vel,  
                               const Vector3& base_angular_vel, const Scalar dt=1.0) {
    q_integ_.template head<3>() = base_pos;
    q_integ_.template segment<4>(3) = base_quat;
    v_integ_.template head<3>() = dt * base_linear_vel;
    v_integ_.template segment<3>(3) = dt * base_angular_vel;
    q_integ_out_ = pinocchio::integrate(model_, q_integ_, v_integ_);
  }

  void updateBaseKinematics(const Vector3& base_pos, const Vector4& base_quat,
                            const Vector3& base_linear_vel,  
                            const Vector3& base_angular_vel, 
                            const Vector3& base_linear_acc, const Scalar dt) {
    q_integ_.template head<3>() = base_pos;
    q_integ_.template segment<4>(3) = base_quat;
    v_integ_.template head<3>() = dt * base_linear_vel;
    v_integ_.template segment<3>(3) = dt * base_angular_vel;
    q_integ_out_ = pinocchio::integrate(model_, q_integ_, v_integ_);
    jac_integ_dq_.template topLeftCorner<6, 6>().setZero();
    pinocchio::dIntegrate(model_, q_integ_, v_integ_, jac_integ_dq_, 
                          pinocchio::ArgumentPosition::ARG0);
    v_integ_.template head<3>() = base_linear_vel;
    v_integ_.template segment<3>(3) = base_angular_vel;
    jac_integ_dv_.template topLeftCorner<6, 6>().setZero();
    pinocchio::dIntegrate(model_, q_integ_, v_integ_, jac_integ_dv_, 
                          pinocchio::ArgumentPosition::ARG1);
    v_base_linear_integ_out_.noalias() = base_linear_vel + dt * base_linear_acc;
  }

  void updateLegKinematics(const Vector3& base_angular_vel,
                           const Vector12& qJ, const Vector12& dqJ,
                           const pinocchio::ReferenceFrame rf=pinocchio::LOCAL_WORLD_ALIGNED) {
    updateKinematics(Vector3::Zero(), Quaternion::Identity().coeffs(), 
                     Vector3::Zero(), base_angular_vel, qJ, dqJ, rf);
  }

  void updateKinematics(const Vector3& base_pos, const Vector4& base_quat, 
                        const Vector3& base_linear_vel,  const Vector3& base_angular_vel, 
                        const Vector12& qJ, const Vector12& dqJ,
                        const pinocchio::ReferenceFrame rf=pinocchio::LOCAL_WORLD_ALIGNED) {
    q_.template head<3>() = base_pos;
    q_.template segment<4>(3) = base_quat;
    q_.template tail<12>() = qJ;
    v_.template head<3>() = base_linear_vel;
    v_.template segment<3>(3) = base_angular_vel;
    v_.template tail<12>() = dqJ;
    pinocchio::normalize(model_, q_);
    pinocchio::forwardKinematics(model_, data_, q_, v_);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::computeJointJacobians(model_, data_, q_);
    for (int i=0; i<contact_frames_.size(); ++i) {
      pinocchio::getFrameJacobian(model_, data_, contact_frames_[i], rf, jac_6d_[i]);
      contact_frame_velocity_[i].noalias() 
          = pinocchio::getFrameVelocity(model_, data_, contact_frames_[i], rf).linear()
              - pinocchio::getFrameVelocity(model_, data_, 1, rf).linear();
    }
  }

  void updateLegDynamics(const Vector12& qJ, const Vector12& dqJ, 
                         const Vector3& base_linear_acc=Vector3::Zero(), 
                         const Vector3& base_angular_acc=Vector3::Zero()) {
    updateDynamics(Vector3::Zero(), Quaternion::Identity().coeffs(), 
                   Vector3::Zero(), Vector3::Zero(), qJ, dqJ,
                   Vector3::Zero(), Vector3::Zero());
  }

  void updateDynamics(const Vector3& base_pos, const Vector4& base_quat, 
                      const Vector3& base_linear_vel, const Vector3& base_angular_vel, 
                      const Vector12& qJ, const Vector12& dqJ,
                      const Vector3& base_linear_acc=Vector3::Zero(), 
                      const Vector3& base_angular_acc=Vector3::Zero()) {
    q_.template head<3>() = base_pos;
    q_.template segment<4>(3) = base_quat;
    q_.template tail<12>() = qJ;
    v_.template head<3>() = base_linear_vel;
    v_.template segment<3>(3) = base_angular_vel;
    v_.template tail<12>() = dqJ;
    a_.template head<3>() = base_linear_acc;
    a_.template segment<3>(3) = base_angular_acc;
    tau_ = pinocchio::rnea(model_, data_, q_, v_, a_);
  }

  const Eigen::VectorBlock<const Vector19, 3> getBasePosition() const {
    return q_integ_out_.template head<3>();
  }

  const Eigen::VectorBlock<const Vector19, 4> getBaseOrientation() const {
    return q_integ_out_.template segment<4>(3);
  }

  const Vector3& getBaseLinearVelocity() const {
    return v_base_linear_integ_out_;
  }

  const Eigen::Block<const Jacobian6D, 6, 6> getBaseJacobianWrtConfiguration() const {
    return jac_integ_dq_.template topLeftCorner<6, 6>();
  }

  const Eigen::Block<const Jacobian6D, 6, 6> getBaseJacobianWrtVelocity() const {
    return jac_integ_dv_.template topLeftCorner<6, 6>();
  }

  const Vector3& getContactPosition(const int contact_id) const {
    return data_.oMf[contact_frames_[contact_id]].translation();
  }

  const Vector3& getContactVelocity(const int contact_id) const {
    return contact_frame_velocity_[contact_id];
  }

  const Eigen::Block<const Jacobian6D, 3, 18> getContactJacobian(const int contact_id) const {
    assert(contat_id >= 0);
    assert(contat_id < 4);
    return jac_6d_[contact_id].template topLeftCorner<3, 18>();
  }

  const Vector18& getDynamics() const {
    return tau_;
  }

  const std::array<int, 4>& contactFrames() const {
    return contact_frames_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  pinocchio::ModelTpl<Scalar> model_;
  pinocchio::DataTpl<Scalar> data_;
  Jacobian6D jac_integ_dq_, jac_integ_dv_;
  Vector19 q_, q_integ_, q_integ_out_;
  Vector18 v_, a_, tau_, v_integ_;
  Vector3 v_base_linear_integ_out_;
  std::vector<Jacobian6D, Eigen::aligned_allocator<Jacobian6D>> jac_6d_;
  std::vector<Vector3> contact_frame_velocity_;
  std::array<int, 4> contact_frames_;

};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_ROBOT_HPP_ 