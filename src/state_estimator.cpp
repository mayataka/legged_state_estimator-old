#include "legged_state_estimator/state_estimator.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

using Vector19d = types::Vector19<double>;
using Vector18d = types::Vector18<double>;
using Vector12d = types::Vector12<double>;
using Vector4d = types::Vector4<double>;
using Vector3d = types::Vector3<double>;
using Quaterniond = types::Quaternion<double>;

using Vector19f = types::Vector19<float>;
using Vector18f = types::Vector18<float>;
using Vector12f = types::Vector12<float>;
using Vector4f = types::Vector4<float>;
using Vector3f = types::Vector3<float>;
using Quaternionf = types::Quaternion<float>;

template StateEstimator<double>::StateEstimator(const StateEstimatorSettings<double>&);
template StateEstimator<float>::StateEstimator(const StateEstimatorSettings<float>&);

template StateEstimator<double>::StateEstimator();
template StateEstimator<float>::StateEstimator();

template StateEstimator<double>::~StateEstimator();
template StateEstimator<float>::~StateEstimator();

template void StateEstimator<double>::resetCalibration();
template void StateEstimator<float>::resetCalibration();

template void StateEstimator<double>::addCalibrationData(const Vector3d&, const Vector3d&);
template void StateEstimator<float>::addCalibrationData(const Vector3f&, const Vector3f&);

template void StateEstimator<double>::doCalibration(const Quaterniond&);
template void StateEstimator<float>::doCalibration(const Quaternionf&);

template void StateEstimator<double>::update(const Quaterniond&, const Vector3d&, const Vector3d&, 
                                             const Vector12d&, const Vector12d&, const Vector4d&);
    // const Vector12d&, const Vector12d&, const std::array<std::int16_t, 4>&);
template void StateEstimator<float>::update(const Quaternionf&, const Vector3f&, const Vector3f&, 
                                            const Vector12f&, const Vector12f&, const Vector4f&);
    // const Vector12f&, const Vector12f&, const std::array<std::int16_t, 4>&);

template void StateEstimator<double>::reset(const double, const double, const Vector4d&);
template void StateEstimator<float>::reset(const float, const float, const Vector4f&);

template const Vector3d& StateEstimator<double>::getBaseLinearVelocityEstimate() const;
template const Vector3f& StateEstimator<float>::getBaseLinearVelocityEstimate() const;

template const Vector3d& StateEstimator<double>::getBaseAngularVelocityEstimate() const;
template const Vector3f& StateEstimator<float>::getBaseAngularVelocityEstimate() const;

template const Vector3d& StateEstimator<double>::getBasePositionEstimate() const;
template const Vector3f& StateEstimator<float>::getBasePositionEstimate() const;

template const Vector12d& StateEstimator<double>::getJointVelocityEstimate() const;
template const Vector12f& StateEstimator<float>::getJointVelocityEstimate() const;

template const Vector4d& StateEstimator<double>::getContactForceEstimate() const;
template const Vector4f& StateEstimator<float>::getContactForceEstimate() const;

template const Vector4d& StateEstimator<double>::getContactProbability() const;
template const Vector4f& StateEstimator<float>::getContactProbability() const;

template double StateEstimator<double>::getNonContactProbability() const;
template float StateEstimator<float>::getNonContactProbability() const;

template const std::array<Vector3d, 4>& StateEstimator<double>::getContactFramePositionEstimate() const;
template const std::array<Vector3f, 4>& StateEstimator<float>::getContactFramePositionEstimate() const;

template const Vector3d& StateEstimator<double>::getContactFramePositionEstimate(const int) const;
template const Vector3f& StateEstimator<float>::getContactFramePositionEstimate(const int) const;

template class StateEstimator<double>;
template class StateEstimator<float>;

} // namespace legged_state_estimator
