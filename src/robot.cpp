#include "legged_state_estimator/robot.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

using Vector19d = types::Vector19<double>;
using Vector18d = types::Vector18<double>;
using Vector16d = types::Vector16<double>;
using Vector12d = types::Vector12<double>;
using Vector4d = types::Vector4<double>;
using Vector3d = types::Vector3<double>;
using Quaterniond = types::Quaternion<double>;
using Jacobian6Dd = types::Matrix<double, 6, 18>;

using Vector19f = types::Vector19<float>;
using Vector18f = types::Vector18<float>;
using Vector16f = types::Vector16<float>;
using Vector12f = types::Vector12<float>;
using Vector4f = types::Vector4<float>;
using Vector3f = types::Vector3<float>;
using Quaternionf = types::Quaternion<float>;
using Jacobian6Df = types::Matrix<float, 6, 18>;

template Robot<double>::Robot(const std::string&, const std::array<int, 4>&);
template Robot<float>::Robot(const std::string&, const std::array<int, 4>&);

template Robot<double>::Robot();
template Robot<float>::Robot();

template Robot<double>::~Robot();
template Robot<float>::~Robot();

template void Robot<double>::updateBaseConfiguration(const Vector3d&, const Vector4d&, 
                                                     const Vector3d&, const Vector3d&, 
                                                     const double);
template void Robot<float>::updateBaseConfiguration(const Vector3f&, const Vector4f&, 
                                                    const Vector3f&, const Vector3f&, 
                                                    const float);

template void Robot<double>::updateBaseKinematics(const Vector3d&, const Vector4d&, 
                                                  const Vector3d&, const Vector3d&, 
                                                  const Vector3d&, const double);
template void Robot<float>::updateBaseKinematics(const Vector3f&, const Vector4f&, 
                                                 const Vector3f&, const Vector3f&, 
                                                 const Vector3f&, const float);

template void Robot<double>::updateLegKinematics(const Vector3d&, const Vector12d&, 
                                                 const Vector12d&, const pinocchio::ReferenceFrame);
template void Robot<float>::updateLegKinematics(const Vector3f&, const Vector12f&,
                                                const Vector12f&, const pinocchio::ReferenceFrame);

template void Robot<double>::updateKinematics(const Vector3d&, const Vector4d&, 
                                              const Vector3d&, const Vector3d&, 
                                              const Vector12d&, const Vector12d&, 
                                              const pinocchio::ReferenceFrame);
template void Robot<float>::updateKinematics(const Vector3f&, const Vector4f&, 
                                             const Vector3f&, const Vector3f&, 
                                             const Vector12f&, const Vector12f&, 
                                             const pinocchio::ReferenceFrame);

template void Robot<double>::updateLegDynamics(const Vector12d&, const Vector12d&, 
                                               const Vector3d&, const Vector3d&);
template void Robot<float>::updateLegDynamics(const Vector12f&, const Vector12f&, 
                                              const Vector3f&, const Vector3f&);

template void Robot<double>::updateDynamics(const Vector3d&, const Vector4d&,
                                            const Vector3d&, const Vector3d&, 
                                            const Vector12d&, const Vector12d&, 
                                            const Vector3d&, const Vector3d&);
template void Robot<float>::updateDynamics(const Vector3f&, const Vector4f&,
                                           const Vector3f&, const Vector3f&, 
                                           const Vector12f&, const Vector12f&, 
                                           const Vector3f&, const Vector3f&);

template const Eigen::VectorBlock<const Vector19d, 3> Robot<double>::getBasePosition() const;
template const Eigen::VectorBlock<const Vector19f, 3> Robot<float>::getBasePosition() const;

template const Eigen::VectorBlock<const Vector19d, 4> Robot<double>::getBaseOrientation() const;
template const Eigen::VectorBlock<const Vector19f, 4> Robot<float>::getBaseOrientation() const;

template const Vector3d& Robot<double>::getBaseLinearVelocity() const;
template const Vector3f& Robot<float>::getBaseLinearVelocity() const;

template const Eigen::Block<const Jacobian6Dd, 6, 6> Robot<double>::getBaseJacobianWrtConfiguration() const;
template const Eigen::Block<const Jacobian6Df, 6, 6> Robot<float>::getBaseJacobianWrtConfiguration() const;

template const Eigen::Block<const Jacobian6Dd, 6, 6> Robot<double>::getBaseJacobianWrtVelocity() const;
template const Eigen::Block<const Jacobian6Df, 6, 6> Robot<float>::getBaseJacobianWrtVelocity() const;

template const Vector3d& Robot<double>::getContactPosition(const int) const;
template const Vector3f& Robot<float>::getContactPosition(const int) const;

template const Vector3d& Robot<double>::getContactVelocity(const int) const;
template const Vector3f& Robot<float>::getContactVelocity(const int) const;

template const Eigen::Block<const Jacobian6Dd, 3, 18> Robot<double>::getContactJacobian(const int) const;
template const Eigen::Block<const Jacobian6Df, 3, 18> Robot<float>::getContactJacobian(const int) const;

template const Vector18d& Robot<double>::getDynamics() const;
template const Vector18f& Robot<float>::getDynamics() const;

template const Eigen::VectorBlock<const Vector18d, 12> Robot<double>::getJointDynamics() const;
template const Eigen::VectorBlock<const Vector18f, 12> Robot<float>::getJointDynamics() const;

template const std::array<int, 4>& Robot<double>::contactFrames() const;
template const std::array<int, 4>& Robot<float>::contactFrames() const;

template class Robot<double>;
template class Robot<float>;

} // namespace legged_state_estimator
