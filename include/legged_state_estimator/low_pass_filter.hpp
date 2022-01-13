#ifndef LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER_HPP_
#define LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER_HPP_ 

#include <cmath>
#include <stdexcept>
#include <iostream>

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

template <typename Scalar, int dim>
class LowPassFilter {
public:
  using Vector = types::Vector<Scalar, dim>;

  LowPassFilter(const Scalar sampling_time, const Scalar cutoff_freq)
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

  LowPassFilter()
    : est_(Vector::Zero()),
      alpha_(0.0) {
  }

  ~LowPassFilter() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(LowPassFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(LowPassFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(LowPassFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(LowPassFilter);

  void reset() {
    est_.setZero();
  }

  void reset(const Vector& est) {
    est_ = est;
  }

  void update(const Vector& obs) {
    est_.array() *= alpha_;
    est_.noalias() += (1.0-alpha_) * obs;
  }

  const Vector& getEstimate() const {
    return est_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Vector est_;
  Scalar alpha_;

};

}

#endif // LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER_HPP_ 