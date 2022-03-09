#ifndef LEGGED_STATE_ESTIMATOR_ROBOT_HPP_
#define LEGGED_STATE_ESTIMATOR_ROBOT_HPP_

#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/StdVector"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include "legged_state_estimator/macros.hpp"


namespace legged_state_estimator {

class Robot {
public:
  Robot(const std::string& path_to_urdf, const std::vector<int>& contact_frames);

  Robot();

  ~Robot();

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(Robot);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(Robot);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(Robot);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(Robot);

  void updateLegKinematics(const Eigen::VectorXd& qJ, const Eigen::VectorXd& dqJ,
                           const pinocchio::ReferenceFrame rf=pinocchio::LOCAL_WORLD_ALIGNED);

  void updateKinematics(const Eigen::Vector3d& base_pos, 
                        const Eigen::Vector4d& base_quat, 
                        const Eigen::Vector3d& base_linear_vel, 
                        const Eigen::Vector3d& base_angular_vel, 
                        const Eigen::VectorXd& qJ, const Eigen::VectorXd& dqJ,
                        const pinocchio::ReferenceFrame rf=pinocchio::LOCAL_WORLD_ALIGNED);

  void updateLegDynamics(const Eigen::VectorXd& qJ, const Eigen::VectorXd& dqJ, 
                         const Eigen::VectorXd& ddqJ);

  void updateDynamics(const Eigen::Vector3d& base_pos, 
                      const Eigen::Vector4d& base_quat, 
                      const Eigen::Vector3d& base_linear_vel, 
                      const Eigen::Vector3d& base_angular_vel, 
                      const Eigen::Vector3d& base_linear_acc, 
                      const Eigen::Vector3d& base_angular_acc,
                      const Eigen::VectorXd& qJ, const Eigen::VectorXd& dqJ,
                      const Eigen::VectorXd& ddqJ);

  const Eigen::Vector3d& getBasePosition() const;

  const Eigen::Matrix3d& getBaseRotation() const;

  const Eigen::Vector3d& getContactPosition(const int contact_id) const;

  const Eigen::Matrix3d& getContactRotation(const int contact_id) const;

  const Eigen::Block<const Eigen::MatrixXd> getContactJacobian(const int contact_id) const;

  const Eigen::Block<const Eigen::MatrixXd> getJointContactJacobian(const int contact_id) const;

  const Eigen::VectorXd& getInverseDynamics() const;

  const Eigen::VectorBlock<const Eigen::VectorXd> getJointInverseDynamics() const;

  const std::vector<int>& getContactFrames() const;

  int nq() const;

  int nv() const;

  int nJ() const;

  int numContacts() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_, v_, a_, tau_;
  std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> jac_6d_;
  std::vector<int> contact_frames_;

};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_ROBOT_HPP_ 