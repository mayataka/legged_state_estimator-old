#include "legged_state_estimator/robot.hpp"


namespace legged_state_estimator {

Robot::Robot(const std::string& path_to_urdf, 
             const std::vector<int>& contact_frames) 
  : contact_frames_(contact_frames),
    model_(),
    data_(),
    q_(),
    v_(),
    a_(),
    tau_(),
    jac_6d_() {
  pinocchio::urdf::buildModel(path_to_urdf, 
                              pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);
  q_   = Eigen::VectorXd(model_.nq);
  v_   = Eigen::VectorXd(model_.nv);
  a_   = Eigen::VectorXd(model_.nv);
  tau_ = Eigen::VectorXd(model_.nv);
  for (int i=0; i<contact_frames.size(); ++i) {
    jac_6d_.push_back(Eigen::MatrixXd::Zero(6, model_.nv));
  }
}


Robot::Robot(const std::string& path_to_urdf, 
             const std::vector<std::string>& contact_frames) 
  : contact_frames_(),
    model_(),
    data_(),
    q_(),
    v_(),
    a_(),
    tau_(),
    jac_6d_() {
  pinocchio::urdf::buildModel(path_to_urdf, 
                              pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);
  q_   = Eigen::VectorXd(model_.nq);
  v_   = Eigen::VectorXd(model_.nv);
  a_   = Eigen::VectorXd(model_.nv);
  tau_ = Eigen::VectorXd(model_.nv);
  for (int i=0; i<contact_frames.size(); ++i) {
    jac_6d_.push_back(Eigen::MatrixXd::Zero(6, model_.nv));
  }
  contact_frames_.clear();
  for (const auto& e : contact_frames) {
    try {
      if (!model_.existFrame(e)) {
        throw std::invalid_argument(
            "Invalid argument: frame " + e + " does not exit!");
      }
    }
    catch(const std::exception& e) {
      std::cerr << e.what() << '\n';
      std::exit(EXIT_FAILURE);
    }
    contact_frames_.push_back(model_.getFrameId(e));
  }
}


Robot::Robot() 
  : contact_frames_(),
    model_(),
    data_(),
    q_(),
    v_(),
    a_(),
    tau_(),
    jac_6d_() {
}


Robot::~Robot() {}


void Robot::updateLegKinematics(const Eigen::VectorXd& qJ, 
                                const Eigen::VectorXd& dqJ,
                                const pinocchio::ReferenceFrame rf) {
  updateKinematics(Eigen::Vector3d::Zero(), 
                   Eigen::Quaterniond::Identity().coeffs(), 
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 
                   qJ, dqJ, rf);
}


void Robot::updateKinematics(const Eigen::Vector3d& base_pos, 
                             const Eigen::Vector4d& base_quat, 
                             const Eigen::Vector3d& base_linear_vel, 
                             const Eigen::Vector3d& base_angular_vel, 
                             const Eigen::VectorXd& qJ, 
                             const Eigen::VectorXd& dqJ, 
                             const pinocchio::ReferenceFrame rf) {
  q_.template head<3>()     = base_pos;
  q_.template segment<4>(3) = base_quat;
  v_.template head<3>()     = base_linear_vel;
  v_.template segment<3>(3) = base_angular_vel;
  q_.tail(model_.nq-7) = qJ;
  v_.tail(model_.nv-6) = dqJ;
  pinocchio::normalize(model_, q_);
  pinocchio::forwardKinematics(model_, data_, q_, v_);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeJointJacobians(model_, data_, q_);
  for (int i=0; i<contact_frames_.size(); ++i) {
    pinocchio::getFrameJacobian(model_, data_, contact_frames_[i], rf, jac_6d_[i]);
  }
}


void Robot::updateLegDynamics(const Eigen::VectorXd& qJ, 
                              const Eigen::VectorXd& dqJ, 
                              const Eigen::VectorXd& ddqJ) {
  updateDynamics(Eigen::Vector3d::Zero(), 
                 Eigen::Quaterniond::Identity().coeffs(), 
                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 
                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 
                 qJ, dqJ, ddqJ);
}


void Robot::updateDynamics(const Eigen::Vector3d& base_pos, 
                           const Eigen::Vector4d& base_quat, 
                           const Eigen::Vector3d& base_linear_vel, 
                           const Eigen::Vector3d& base_angular_vel, 
                           const Eigen::Vector3d& base_linear_acc, 
                           const Eigen::Vector3d& base_angular_acc,
                           const Eigen::VectorXd& qJ, 
                           const Eigen::VectorXd& dqJ,
                           const Eigen::VectorXd& ddqJ) {
  q_.template head<3>()     = base_pos;
  q_.template segment<4>(3) = base_quat;
  v_.template head<3>()     = base_linear_vel;
  v_.template segment<3>(3) = base_angular_vel;
  a_.template head<3>()     = base_linear_acc;
  a_.template segment<3>(3) = base_angular_acc;
  q_.tail(model_.nq-7) = qJ;
  v_.tail(model_.nv-6) = dqJ;
  a_.tail(model_.nv-6) = ddqJ;
  tau_ = pinocchio::rnea(model_, data_, q_, v_, a_);
}


const Eigen::Vector3d& Robot::getBasePosition() const {
  return data_.oMf[1].translation();
}


const Eigen::Matrix3d& Robot::getBaseRotation() const {
  return data_.oMf[1].rotation();
}


const Eigen::Vector3d& Robot::getContactPosition(const int contact_id) const {
  assert(contat_id >= 0);
  assert(contat_id < contact_frames_.size());
  return data_.oMf[contact_frames_[contact_id]].translation();
}


const Eigen::Matrix3d& Robot::getContactRotation(const int contact_id) const {
  assert(contat_id >= 0);
  assert(contat_id < contact_frames_.size());
  return data_.oMf[contact_frames_[contact_id]].rotation();
}


const Eigen::Block<const Eigen::MatrixXd> Robot::getContactJacobian(const int contact_id) const {
  assert(contat_id >= 0);
  assert(contat_id < contact_frames_.size());
  return jac_6d_[contact_id].topRows(3);
}


const Eigen::Block<const Eigen::MatrixXd> Robot::getJointContactJacobian(const int contact_id) const {
  assert(contat_id >= 0);
  assert(contat_id < contact_frames_.size());
  return jac_6d_[contact_id].topRightCorner(3, nJ());
}


const Eigen::VectorXd& Robot::getInverseDynamics() const {
  return tau_;
}


const Eigen::VectorBlock<const Eigen::VectorXd> Robot::getJointInverseDynamics() const {
  return tau_.tail(nJ());
}


const std::vector<int>& Robot::getContactFrames() const {
  return contact_frames_;
}


int Robot::nq() const {
  return model_.nq;
}


int Robot::nv() const {
  return model_.nv;
}


int Robot::nJ() const {
  return model_.nv-6;
}


int Robot::numContacts() const {
  return contact_frames_.size();
}

} // namespace legged_state_estimator
