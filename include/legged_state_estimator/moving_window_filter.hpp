#ifndef LEGGED_STATE_ESTIMATOR_MOVING_WINDOW_FILTER_HPP_
#define LEGGED_STATE_ESTIMATOR_MOVING_WINDOW_FILTER_HPP_

#include <stdexcept>
#include <iostream>
#include <deque>
#include <array>

#include "Eigen/StdDeque"

#include "legged_state_estimator/macros.hpp"


namespace legged_state_estimator {

template <typename Scalar, int dim>
class MovingWindowFilter {
public:
  using Vector = Eigen::Matrix<Scalar, dim, 1>;

  MovingWindowFilter(const std::size_t window_size=1)
    : window_size_(window_size),
      average_(Vector::Zero()),
      correction_(Vector::Zero()),
      sum_(Vector::Zero()),
      tmp_(Vector::Zero()),
      history_() {
    history_.push_back(Vector::Zero());
    try {
      if (window_size <= 0) {
        throw std::out_of_range(
            "Invalid argment: window_size must be positive!");
      }
    }
    catch(const std::exception& e) {
      std::cerr << e.what() << '\n';
      std::exit(EXIT_FAILURE);
    }
  }

  ~MovingWindowFilter() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(MovingWindowFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(MovingWindowFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(MovingWindowFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(MovingWindowFilter);

  void reset() {
    average_.setZero();
    correction_.setZero();
    sum_.setZero();
    tmp_.setZero();
    history_.clear();
  }

  void update(const Vector& new_obs) {
    if (history_.size() >= window_size_) {
      tmp_ = - history_.front();
      updateSum(tmp_);
      history_.pop_front();
    }
    updateSum(new_obs);
    history_.push_back(new_obs);
    average_ = (sum_ + correction_) / history_.size();
  }

  const Vector& getEstimate() const {
    return average_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  int window_size_;
  Vector average_, correction_, sum_, tmp_;
  std::deque<Vector, Eigen::aligned_allocator<Vector>> history_;


  void updateSum(const Vector& new_obs) {
    for (int i=0; i<dim; ++i) {
      const Scalar new_sum = sum_.coeff(i) + new_obs.coeff(i);
      if (std::abs(sum_.coeff(i)) > std::abs(new_obs.coeff(i))) {
        correction_.coeffRef(i) += (sum_.coeff(i) - new_sum) + new_obs.coeff(i);
      }
      else {
        correction_.coeffRef(i) += (new_obs.coeff(i) - new_sum) + sum_.coeff(i);
      }
      sum_.coeffRef(i) = new_sum;
    }
  }

};

}

#endif // LEGGED_STATE_ESTIMATOR_MOVING_WINDOW_FILTER_HPP_