#include "legged_state_estimator/contact_estimator.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

using Vector4d = types::Vector4<double>;
using Vector4f = types::Vector4<float>;

template ContactEstimator<double>::ContactEstimator(const Vector4d&, const int, const double, const double);
template ContactEstimator<float>::ContactEstimator(const Vector4f&, const int, const float, const float);

template ContactEstimator<double>::ContactEstimator();
template ContactEstimator<float>::ContactEstimator();

template ContactEstimator<double>::~ContactEstimator();
template ContactEstimator<float>::~ContactEstimator();

template void ContactEstimator<double>::reset();
template void ContactEstimator<float>::reset();

template void ContactEstimator<double>::update(const Vector4d&);
template void ContactEstimator<float>::update(const Vector4f&);

template const Vector4d& ContactEstimator<double>::getContactForceEstimate() const;
template const Vector4f& ContactEstimator<float>::getContactForceEstimate() const;

template const Vector4d& ContactEstimator<double>::getContactProbability() const;
template const Vector4f& ContactEstimator<float>::getContactProbability() const;

template double ContactEstimator<double>::getContactProbability(const int) const;
template float ContactEstimator<float>::getContactProbability(const int) const;

template double ContactEstimator<double>::getNonContactProbability() const;
template float ContactEstimator<float>::getNonContactProbability() const;

template void ContactEstimator<double>::setForceBias(const Vector4d&);
template void ContactEstimator<float>::setForceBias(const Vector4f&);

template const Vector4d& ContactEstimator<double>::getForceBias() const;
template const Vector4f& ContactEstimator<float>::getForceBias() const;

template class ContactEstimator<double>;
template class ContactEstimator<float>;

} // namespace legged_state_estimator
