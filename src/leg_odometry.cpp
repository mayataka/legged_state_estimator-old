#include "legged_state_estimator/leg_odometry.hpp"
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

template LegOdometry<double>::LegOdometry(
    const std::string&, const std::array<int, 4>&, const double, const double);
template LegOdometry<float>::LegOdometry(
    const std::string&, const std::array<int, 4>&, const float, const float);

template LegOdometry<double>::LegOdometry();
template LegOdometry<float>::LegOdometry();

template LegOdometry<double>::~LegOdometry();
template LegOdometry<float>::~LegOdometry();

template void LegOdometry<double>::updateBaseStateEstimation(
    const Quaterniond&, const Vector3d&, const Vector12d&, const Vector12d&, const Vector4d&);
template void LegOdometry<float>::updateBaseStateEstimation(
    const Quaternionf&, const Vector3f&, const Vector12f&, const Vector12f&, const Vector4f&);

template void LegOdometry<double>::updateContactPositionEstimation(
    const Vector3d&, const Vector4d&);
template void LegOdometry<float>::updateContactPositionEstimation(
    const Vector3f&, const Vector4f&);

template void LegOdometry<double>::resetBaseStateEstimation(
    const double, const double, const Vector4d&);
template void LegOdometry<float>::resetBaseStateEstimation(
    const float, const float, const Vector4f&);

template void LegOdometry<double>::resetContactPositionEstimation(
    const double, const double, const Vector4d&);
template void LegOdometry<float>::resetContactPositionEstimation(
    const float, const float, const Vector4f&);

template const Vector3d& LegOdometry<double>::getBaseLinearVelocityEstimate() const;
template const Vector3f& LegOdometry<float>::getBaseLinearVelocityEstimate() const;

template const Vector3d& LegOdometry<double>::getBasePositionEstimate() const;
template const Vector3f& LegOdometry<float>::getBasePositionEstimate() const;

template const std::array<Vector3d, 4>& LegOdometry<double>::getContactFramePositionEstimate() const;
template const std::array<Vector3f, 4>& LegOdometry<float>::getContactFramePositionEstimate() const;

template const Vector3d& LegOdometry<double>::getContactFramePositionEstimate(const int) const;
template const Vector3f& LegOdometry<float>::getContactFramePositionEstimate(const int) const;

template const std::array<Vector3d, 4>& LegOdometry<double>::getContactFrameVelocityEstimate() const;
template const std::array<Vector3f, 4>& LegOdometry<float>::getContactFrameVelocityEstimate() const;

template const Vector3d& LegOdometry<double>::getContactFrameVelocityEstimate(const int) const;
template const Vector3f& LegOdometry<float>::getContactFrameVelocityEstimate(const int) const;

template class LegOdometry<double>;
template class LegOdometry<float>;
  
} // namespace legged_state_estimator
