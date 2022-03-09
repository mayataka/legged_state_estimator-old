#ifndef LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_
#define LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_

#include <vector>
#include <utility>
#include <cmath>
#include <stdexcept>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/StdVector"

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/robot.hpp"
#include "legged_state_estimator/schmitt_trigger.hpp"


namespace legged_state_estimator {

struct ContactEstimatorSettings {
  double beta0;
  double beta1;
  std::vector<double> force_sensor_bias;
  SchmittTriggerSettings schmitt_trigger_settings;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


class ContactEstimator {
public:
  ContactEstimator(const Robot& robot, 
                   const ContactEstimatorSettings& settings);

  ContactEstimator();

  ~ContactEstimator();

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(ContactEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(ContactEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(ContactEstimator);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(ContactEstimator);

  void reset();

  void update(Robot& robot, const Eigen::VectorXd& tauJ, 
              const std::vector<double>& force_sensor_raw);

  void setParameters(const ContactEstimatorSettings& settings);

  std::vector<std::pair<int, bool>> getContactState(const double prob_threshold=0.5) const;

  const std::vector<Eigen::Vector3d>& getContactForceEstimate() const;

  const std::vector<double>& getContactForceNormalEstimate() const;

  const std::vector<double>& getContactProbability() const;

  const std::vector<double>& getForceSensorBias() const;

  const std::vector<Eigen::Vector3d>& getContactSurfaceNormal() const;

  void setForceSensorBias(const std::vector<double>& force_sensor_bias);

  void setContactSurfaceNormal(const std::vector<Eigen::Vector3d>& contact_surface_normal);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  ContactEstimatorSettings settings_;
  std::vector<Eigen::Vector3d> contact_force_estimate_, contact_surface_normal_;
  std::vector<double> contact_force_normal_estimate_, contact_probability_;
  std::vector<SchmittTrigger> schmitt_trigger_;
  int num_contacts_;
};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_ 