#ifndef LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_
#define LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_

#include <array>
#include <cmath>
#include <stdexcept>
#include <iostream>

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"
#include "legged_state_estimator/robot.hpp"
#include "legged_state_estimator/complementary_filter.hpp"


namespace legged_state_estimator {

template <typename Scalar>
class ContactEstimator {
public:
  using Jacobian6D = types::Matrix<Scalar, 6, 18>;
  using Vector19   = types::Vector19<Scalar>;
  using Vector18   = types::Vector18<Scalar>;
  using Vector12   = types::Vector12<Scalar>;
  using Vector4    = types::Vector4<Scalar>;
  using Vector3    = types::Vector3<Scalar>;
  using Quaternion = types::Quaternion<Scalar>;

  ContactEstimator(const Scalar sampling_time, const Scalar compl_cutoff_freq,
                   const Scalar beta1_logistic_reg, const Scalar beta0_logistic_reg, 
                   const Vector4& force_sensor_bias)
   : force_sensor_bias_(force_sensor_bias),
     contact_force_normal_est_(Vector4::Zero()),
     contact_probability_(Vector4::Ones()),
     contact_force_est_(),
     contact_surface_normal_(),
     beta1_logistic_reg_(beta1_logistic_reg),
     beta0_logistic_reg_(beta0_logistic_reg),
     non_contact_probability_(0),
     contact_force_filter_(sampling_time, compl_cutoff_freq) {
    try {
      if (sampling_time <= 0) {
        throw std::out_of_range(
            "Invalid argment: sampling_time must be positive!");
      }
      if (compl_cutoff_freq <= 0) {
        throw std::out_of_range(
            "Invalid argment: compl_cutoff_freq must be positive!");
      }
    }
    catch(const std::exception& e) {
      std::cerr << e.what() << '\n';
      std::exit(EXIT_FAILURE);
    }
    for (auto& e : contact_force_est_) {
      e = Vector3::Zero();
    }
    for (auto& e : contact_surface_normal_) {
      e << 0, 0, 1;
    }
  }

  ContactEstimator() 
   : force_sensor_bias_(),
     contact_force_normal_est_(),
     contact_probability_(),
     contact_force_est_(),
     contact_surface_normal_(),
     beta1_logistic_reg_(0),
     beta0_logistic_reg_(0),
     non_contact_probability_(0),
     contact_force_filter_() {
  }

  ~ContactEstimator() {
  }

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(ContactEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(ContactEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(ContactEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(ContactEstimator);

  void update(const Robot<Scalar>& robot, const Vector12& tau, 
              const Vector4& force_sensor_raw) {
    // Estimate contactforce
    for (int i=0; i<4; ++i) {
      contact_force_est_[i].noalias() 
          = - robot.getContactJacobian(i).template block<3, 3>(0, 6+i*3).transpose().inverse() 
              * (tau.template segment<3>(3*i)-robot.getDynamics().template segment<3>(6+3*i));
      contact_force_normal_est_.coeffRef(i) 
          = contact_force_est_[i].dot(contact_surface_normal_[i]);
    }
    contact_force_filter_.update(contact_force_normal_est_, force_sensor_raw);
    contact_force_normal_est_ = contact_force_filter_.getEstimate();
    // From force sensors
    // Contact probability 
    for (int i=0; i<4; ++i) {
      contact_probability_.coeffRef(i) 
          = 1.0 / (1.0 + std::exp(- beta1_logistic_reg_ * contact_force_normal_est_.coeff(i) 
                                  - beta0_logistic_reg_));
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

  void setContactSurfaceNormal(const std::array<Vector3, 4>& contact_surface_normal) {
    contact_surface_normal_ = contact_surface_normal;
  }

  void setContactSurfaceNormal(const Vector3& contact_surface_normal, 
                               const int contact_id) {
    contact_surface_normal_[contact_id] = contact_surface_normal;
  }

  const std::array<Vector3, 4>& getContactForceEstimate() const {
    return contact_force_est_;
  }

  const Vector3& getContactForceEstimate(const int contact_id) const {
    return contact_force_est_[contact_id];
  }

  const Vector4& getContactForceNormalEstimate() const {
    return contact_force_normal_est_;
  }

  Scalar getContactForceNormalEstimate(const int contact_id) const {
    return contact_force_normal_est_.coeff(contact_id);
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

  void setForceSensorBias(const Vector4& force_sensor_bias) {
    force_sensor_bias_ = force_sensor_bias;
  }

  const Vector4& getForceSensorBias() const {
    return force_sensor_bias_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Vector4 force_sensor_bias_, contact_force_normal_est_, contact_probability_;
  std::array<Vector3, 4> contact_force_est_, contact_surface_normal_;
  Scalar beta1_logistic_reg_, beta0_logistic_reg_, non_contact_probability_;
  ComplementaryFilter<Scalar, 4> contact_force_filter_;
};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_ 