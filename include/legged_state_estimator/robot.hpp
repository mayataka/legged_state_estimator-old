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

///
/// @class Robot
/// @brief Dynamics and kinematics model of robots. Wraps pinocchio::Model and 
/// pinocchio::Data. Includes contacts.
///
class Robot {
public:
  ///
  /// @brief Constructs a robot model. Builds the Pinocchio robot model and data 
  /// from URDF. Assumes that the robot never has any contacts.
  /// @param[in] path_to_urdf Path to the URDF file.
  /// @param[in] imu_frames id of the IMU frame.
  /// @param[in] contact_frames Collection of the id of frames that can have 
  /// contacts with the environments. 
  ///
  Robot(const std::string& path_to_urdf, const int imu_frame, 
        const std::vector<int>& contact_frames);

  ///
  /// @brief Constructs a robot model. Builds the Pinocchio robot model and data 
  /// from URDF. Assumes that the robot never has any contacts.
  /// @param[in] path_to_urdf Path to the URDF file.
  /// @param[in] imu_frames Name of the IMU frame.
  /// @param[in] contact_frames Collection of the names of frames that can have 
  /// contacts with the environments. 
  ///
  Robot(const std::string& path_to_urdf, const std::string& imu_frame, 
        const std::vector<std::string>& contact_frames);

  ///
  /// @brief Default constructor. 
  ///
  Robot();

  ///
  /// @brief Destructor. 
  ///
  ~Robot();

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(Robot);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(Robot);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(Robot);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(Robot);

  ///
  /// @brief Updates leg kinemarics.
  /// @param[in] qJ Joint positions. Size must be Robot::nJ().
  /// @param[in] rf Reference frame of the kinematics. Default is 
  /// pinocchio::LOCAL_WORLD_ALIGNED.
  ///
  void updateLegKinematics(const Eigen::VectorXd& qJ,
                           const pinocchio::ReferenceFrame rf=pinocchio::LOCAL_WORLD_ALIGNED);

  ///
  /// @brief Updates leg kinemarics.
  /// @param[in] qJ Joint positions. Size must be Robot::nJ().
  /// @param[in] dqJ Joint velocities. Size must be Robot::nJ().
  /// @param[in] rf Reference frame of the kinematics. Default is 
  /// pinocchio::LOCAL_WORLD_ALIGNED.
  ///
  void updateLegKinematics(const Eigen::VectorXd& qJ, const Eigen::VectorXd& dqJ,
                           const pinocchio::ReferenceFrame rf=pinocchio::LOCAL_WORLD_ALIGNED);

  ///
  /// @brief Updates kinemarics.
  /// @param[in] base_pos Base position. 
  /// @param[in] base_quat Base orientation expressed by quaternion (x, y, z, w). 
  /// @param[in] qJ Joint positions. Size must be Robot::nJ().
  /// @param[in] rf Reference frame of the kinematics. Default is 
  /// pinocchio::LOCAL_WORLD_ALIGNED.
  ///
  void updateKinematics(const Eigen::Vector3d& base_pos, 
                        const Eigen::Vector4d& base_quat, 
                        const Eigen::VectorXd& qJ, 
                        const pinocchio::ReferenceFrame rf=pinocchio::LOCAL_WORLD_ALIGNED);

  ///
  /// @brief Updates kinemarics.
  /// @param[in] base_pos Base position. 
  /// @param[in] base_quat Base orientation expressed by quaternion (x, y, z, w). 
  /// @param[in] base_linear_vel Base linear velocity expressed in the body 
  /// local coordinate. 
  /// @param[in] base_angular_vel Base angular velocity expressed in the body 
  /// local coordinate. 
  /// @param[in] qJ Joint positions. Size must be Robot::nJ().
  /// @param[in] dqJ Joint velocities. Size must be Robot::nJ().
  /// @param[in] rf Reference frame of the kinematics. Default is 
  /// pinocchio::LOCAL_WORLD_ALIGNED.
  ///
  void updateKinematics(const Eigen::Vector3d& base_pos, 
                        const Eigen::Vector4d& base_quat, 
                        const Eigen::Vector3d& base_linear_vel, 
                        const Eigen::Vector3d& base_angular_vel, 
                        const Eigen::VectorXd& qJ, const Eigen::VectorXd& dqJ,
                        const pinocchio::ReferenceFrame rf=pinocchio::LOCAL_WORLD_ALIGNED);

  ///
  /// @brief Updates leg dynamics.
  /// @param[in] qJ Joint positions. Size must be Robot::nJ().
  /// @param[in] dqJ Joint velocities. Size must be Robot::nJ().
  ///
  void updateLegDynamics(const Eigen::VectorXd& qJ, const Eigen::VectorXd& dqJ);

  ///
  /// @brief Updates dynamics.
  /// @param[in] base_pos Base position. 
  /// @param[in] base_quat Base orientation expressed by quaternion (x, y, z, w). 
  /// @param[in] base_linear_vel Base linear velocity expressed in the body 
  /// local coordinate. 
  /// @param[in] base_angular_vel Base angular velocity expressed in the body 
  /// local coordinate. 
  /// @param[in] base_linear_vel Base linear acceleration expressed in the body 
  /// local coordinate. 
  /// @param[in] base_angular_vel Base angular acceleration expressed in the body 
  /// local coordinate. 
  /// @param[in] qJ Joint positions. Size must be Robot::nJ().
  /// @param[in] dqJ Joint velocities. Size must be Robot::nJ().
  /// @param[in] ddqJ Joint accelerations. Size must be Robot::nJ().
  ///
  void updateDynamics(const Eigen::Vector3d& base_pos, 
                      const Eigen::Vector4d& base_quat, 
                      const Eigen::Vector3d& base_linear_vel, 
                      const Eigen::Vector3d& base_angular_vel, 
                      const Eigen::Vector3d& base_linear_acc, 
                      const Eigen::Vector3d& base_angular_acc,
                      const Eigen::VectorXd& qJ, const Eigen::VectorXd& dqJ,
                      const Eigen::VectorXd& ddqJ);

  ///
  /// @return const reference to the base position.
  ///
  const Eigen::Vector3d& getBasePosition() const;

  ///
  /// @return const reference to the base orientation expressed by a rotation 
  /// matrix.
  ///
  const Eigen::Matrix3d& getBaseRotation() const;

  ///
  /// @return const reference to the contact position. 
  ///
  const Eigen::Vector3d& getContactPosition(const int contact_id) const;

  ///
  /// @return const reference to the contact orientation expressed by a rotation
  /// matrix. 
  ///
  const Eigen::Matrix3d& getContactRotation(const int contact_id) const;

  ///
  /// @return const reference to the contact Jacobian with respect to the 
  /// generalized coordinate (base pos + base orn + joint positions). 
  /// Size is 3 x Robot::nv().
  ///
  const Eigen::Block<const Eigen::MatrixXd> getContactJacobian(const int contact_id) const;

  ///
  /// @return const reference to the contact Jacobian with respect to the 
  /// joint positions. Size is 3 x Robot::nJ().
  ///
  const Eigen::Block<const Eigen::MatrixXd> getJointContactJacobian(const int contact_id) const;

  ///
  /// @return const reference to the inverse dynamics. Size is Robot::nv().
  ///
  const Eigen::VectorXd& getInverseDynamics() const;

  ///
  /// @return const reference to the inverse dynamics at the joint. Size is 
  /// Robot::nJ().
  /// 
  const Eigen::VectorBlock<const Eigen::VectorXd> getJointInverseDynamics() const;

  ///
  /// @return const reference to the contact frames.
  ///
  const std::vector<int>& getContactFrames() const;

  ///
  /// @return Dimension of the configuration.
  ///
  int nq() const;

  ///
  /// @return Dimension of the generalized velocity.
  ///
  int nv() const;

  ///
  /// @return Number of the joints.
  ///
  int nJ() const;

  ///
  /// @return Number of the contacts.
  ///
  int numContacts() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_, v_, a_, tau_;
  std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> jac_6d_;
  int imu_frame_;
  std::vector<int> contact_frames_;

};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_ROBOT_HPP_ 