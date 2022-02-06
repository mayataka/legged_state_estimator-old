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

using Vector19f = types::Vector19<float>;
using Vector18f = types::Vector18<float>;
using Vector16f = types::Vector16<float>;
using Vector12f = types::Vector12<float>;
using Vector4f = types::Vector4<float>;
using Vector3f = types::Vector3<float>;
using Quaternionf = types::Quaternion<float>;

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

// template const Eigen::VectorBlock<const Vector16d, 3> Robot<double>::getBaseLinearVelocityEstimate() const;
// template const Eigen::VectorBlock<const Vector16f, 3> Robot<float>::getBaseLinearVelocityEstimate() const;

// template const Vector3d& Robot<double>::getBaseAngularVelocityEstimate() const;
// template const Vector3f& Robot<float>::getBaseAngularVelocityEstimate() const;

// template const Vector12d& Robot<double>::getJointVelocityEstimate() const;
// template const Vector12f& Robot<float>::getJointVelocityEstimate() const;

// template const Vector12d& Robot<double>::getJointTorqueEstimate() const;
// template const Vector12f& Robot<float>::getJointTorqueEstimate() const;

// template double Robot<double>::getContactProbability(const int) const;
// template float Robot<float>::getContactProbability(const int) const;

// template const Vector4d& Robot<double>::getContactProbability() const;
// template const Vector4f& Robot<float>::getContactProbability() const;

// template double Robot<double>::getNonContactProbability() const;
// template float Robot<float>::getNonContactProbability() const;

template class Robot<double>;
template class Robot<float>;

} // namespace legged_state_estimator
