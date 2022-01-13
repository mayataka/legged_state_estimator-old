#ifndef LEGGED_STATE_ESTIMATOR_HIGH_PASS_FILTER_HPP_
#define LEGGED_STATE_ESTIMATOR_HIGH_PASS_FILTER_HPP_

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"
#include "legged_state_estimator/low_pass_filter.hpp"


namespace legged_state_estimator {

template <typename Scalar, int dim>
class HighPassFilter {
public:
  using Vector = types::Vector<Scalar, dim>;

  HighPassFilter(const Scalar sampling_time, const Scalar cutoff_freq)
    : lpf_(sampling_time, cutoff_freq),
      est_(Vector::Zero()) {
  }

  HighPassFilter()
    : lpf_(),
      est_(Vector::Zero()) {
  }

  ~HighPassFilter() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(HighPassFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(HighPassFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(HighPassFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(HighPassFilter);

  void reset() {
    lpf_.reset();
    est_.setZero();
  }

  void reset(const Vector& est) {
    lpf_.reset(est);
    est_ = est;
  }

  void update(const Vector& obs) {
    lpf_.updateEstimate(obs);
    est_ = obs - lpf_.getEstimate();
  }

  const Vector& getEstimate() const {
    return est_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  LowPassFilter<Scalar, dim> lpf_;
  Vector est_;

};

} // legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_HIGH_PASS_FILTER_HPP_ 