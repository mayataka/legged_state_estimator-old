#include "legged_state_estimator/contact_estimator.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

using Vector12d = types::Vector12<double>;
using Vector12f = types::Vector12<float>;
using Vector4d = types::Vector4<double>;
using Vector4f = types::Vector4<float>;
using Vector3d = types::Vector3<double>;
using Vector3f = types::Vector3<float>;

template ContactEstimator<double>::ContactEstimator(const double, const double, 
                                                    const double, const double, 
                                                    const Vector4d&);
template ContactEstimator<float>::ContactEstimator(const float, const float, 
                                                   const float, const float, 
                                                   const Vector4f&);

template ContactEstimator<double>::ContactEstimator();
template ContactEstimator<float>::ContactEstimator();

template ContactEstimator<double>::~ContactEstimator();
template ContactEstimator<float>::~ContactEstimator();

template void ContactEstimator<double>::update(const Robot<double>&, 
                                               const Vector12d&, const Vector4d&);
template void ContactEstimator<float>::update(const Robot<float>&, 
                                              const Vector12f&, const Vector4f&);

template void ContactEstimator<double>::setContactSurfaceNormal(const std::array<Vector3d, 4>&);
template void ContactEstimator<float>::setContactSurfaceNormal(const std::array<Vector3f, 4>&);

template void ContactEstimator<double>::setContactSurfaceNormal(const Vector3d&, const int);
template void ContactEstimator<float>::setContactSurfaceNormal(const Vector3f&, const int);

template const std::array<Vector3d, 4>& ContactEstimator<double>::getContactForceEstimate() const;
template const std::array<Vector3f, 4>& ContactEstimator<float>::getContactForceEstimate() const;

template const Vector3d& ContactEstimator<double>::getContactForceEstimate(const int) const;
template const Vector3f& ContactEstimator<float>::getContactForceEstimate(const int) const;

template const Vector4d& ContactEstimator<double>::getContactForceNormalEstimate() const;
template const Vector4f& ContactEstimator<float>::getContactForceNormalEstimate() const;

template double ContactEstimator<double>::getContactForceNormalEstimate(const int) const;
template float ContactEstimator<float>::getContactForceNormalEstimate(const int) const;

template const Vector4d& ContactEstimator<double>::getContactProbability() const;
template const Vector4f& ContactEstimator<float>::getContactProbability() const;

template double ContactEstimator<double>::getContactProbability(const int) const;
template float ContactEstimator<float>::getContactProbability(const int) const;

template double ContactEstimator<double>::getNonContactProbability() const;
template float ContactEstimator<float>::getNonContactProbability() const;

template void ContactEstimator<double>::setForceSensorBias(const Vector4d&);
template void ContactEstimator<float>::setForceSensorBias(const Vector4f&);

template const Vector4d& ContactEstimator<double>::getForceSensorBias() const;
template const Vector4f& ContactEstimator<float>::getForceSensorBias() const;

template class ContactEstimator<double>;
template class ContactEstimator<float>;

} // namespace legged_state_estimator
