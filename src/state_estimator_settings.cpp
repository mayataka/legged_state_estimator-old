#include "legged_state_estimator/state_estimator_settings.hpp"


namespace legged_state_estimator {

template StateEstimatorSettings<double> StateEstimatorSettings<double>::A1Settings(const std::string&, const double);
template StateEstimatorSettings<float> StateEstimatorSettings<float>::A1Settings(const std::string&, const float);

template class StateEstimatorSettings<double>;
template class StateEstimatorSettings<float>;

} // namespace legged_state_estimator