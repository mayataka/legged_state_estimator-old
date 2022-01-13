#ifndef LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_
#define LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_

#include <cmath>
#include <stdexcept>
#include <iostream>

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"
#include "legged_state_estimator/moving_window_filter.hpp"


namespace legged_state_estimator {

template <typename Scalar>
class ContactEstimator {
public:
  using Vector4 = types::Vector4<Scalar>;

  ContactEstimator(const Vector4& force_sensor_bias, 
                   const int window_filter_size, 
                   const Scalar beta1, const Scalar beta0)
   : f_bias_(force_sensor_bias),
     f_est_(Vector4::Zero()),
     contact_probability_(Vector4::Zero()),
     beta1_(beta1),
     beta0_(beta0),
     non_contact_probability_(0.0),
     win_filter_(window_filter_size) {
    contact_probability_.fill(1.0);
    try {
      if (window_filter_size <= 0) {
        throw std::out_of_range(
            "Invalid argment: window_filter_size must be positive!");
      }
    }
    catch(const std::exception& e) {
      std::cerr << e.what() << '\n';
      std::exit(EXIT_FAILURE);
    }
  }

  ContactEstimator() 
   : f_bias_(),
     f_est_(),
     contact_probability_(),
     beta1_(0),
     beta0_(0),
     win_filter_() {
  }

  ~ContactEstimator() {
  }

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(ContactEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(ContactEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(ContactEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(ContactEstimator);

  void reset() {
    contact_probability_.fill(0.0);
    non_contact_probability_ = 1.0;
  }

  void update(const Vector4& f_raw) {
    f_est_ = f_raw - f_bias_;
    win_filter_.update(f_est_);
    for (int i=0; i<4; ++i) {
      contact_probability_.coeffRef(i) 
          = 1.0 / (1.0 + std::exp(- beta1_*win_filter_.getEstimate().coeff(i) - beta0_));
      if (std::isnan(contact_probability_.coeffRef(i)) 
            || std::isinf(contact_probability_.coeffRef(i))) {
        contact_probability_.coeffRef(i) = 0;
      }
    }
    non_contact_probability_ = 1.0;
    for (int i=0; i<4; ++i) {
      non_contact_probability_ *= (1.0-contact_probability_.coeff(i));
    }
  }

  const Vector4& getContactForceEstimate() const {
    return f_est_;
  }

  const Vector4& getContactProbability() const {
    return contact_probability_;
  }

  Scalar getContactProbability(const int contact_id) const {
    return contact_probability_.coeff(contact_id);
  }

  Scalar getNonContactProbability() const {
    return non_contact_probability_;
  }

  void setForceBias(const Vector4& force_sensor_bias) {
    f_bias_ = force_sensor_bias;
  }

  const Vector4& getForceBias() const {
    return f_bias_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Vector4 f_bias_, f_est_, contact_probability_;
  Scalar beta1_, beta0_, non_contact_probability_;
  int num_calib_sums_;
  MovingWindowFilter<Scalar, 4> win_filter_;
};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_ 