#ifndef LEGGED_STATE_ESTIMATOR_COMPLEMENTARY_FILTER_HPP_
#define LEGGED_STATE_ESTIMATOR_COMPLEMENTARY_FILTER_HPP_

#include <cmath>
#include <stdexcept>
#include <iostream>
#include <cassert>

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

template <typename Scalar, int dim>
class ComplementaryFilter {
public:
  using Vector = types::Vector<Scalar, dim>;

  ComplementaryFilter(const Scalar sampling_time, const Scalar cutoff_freq)
    : est_(Vector::Zero()),
      alpha_(1.0-std::exp(-sampling_time*2.0*M_PI*cutoff_freq)) {
    try {
      if (sampling_time <= 0) {
        throw std::out_of_range(
            "Invalid argment: sampling_time must be positive!");
      }
      if (cutoff_freq <= 0) {
        throw std::out_of_range(
            "Invalid argment: cutoff_freq must be positive!");
      }
    }
    catch(const std::exception& e) {
      std::cerr << e.what() << '\n';
      std::exit(EXIT_FAILURE);
    }
  }

  ComplementaryFilter()
    : est_(Vector::Zero()),
      alpha_(0.0) {
  }

  ~ComplementaryFilter() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(ComplementaryFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(ComplementaryFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(ComplementaryFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(ComplementaryFilter);

  void reset() {
    est_.setZero();
  }

  void reset(const Vector& est) {
    est_ = est;
  }

  void update(const Vector& obs_hpf, const Vector& obs_lpf) {
    est_.array() *= alpha_;
    est_.noalias() += alpha_ * obs_hpf;
    est_.noalias() += (1.0-alpha_) * obs_lpf;
  }

  const Vector& getEstimate() const {
    return est_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Vector est_;
  Scalar alpha_, dt_;

};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_COMPLEMENTARY_FILTER_HPP_