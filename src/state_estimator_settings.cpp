#include "legged_state_estimator/state_estimator_settings.hpp"


namespace legged_state_estimator {

template StateEstimatorSettings<double> StateEstimatorSettings<double>::defaultSettings(const std::string&, const double);
template StateEstimatorSettings<float> StateEstimatorSettings<float>::defaultSettings(const std::string&, const float);

template class StateEstimatorSettings<double>;
template class StateEstimatorSettings<float>;

} // namespace legged_state_estimator