#include "legged_state_estimator/contact_estimator.hpp"


namespace legged_state_estimator {

ContactEstimator::ContactEstimator(const Robot& robot,
                                   const ContactEstimatorSettings& settings)
  : settings_(settings),
    contact_force_normal_estimate_(robot.numContacts(), 0),
    contact_probability_(robot.numContacts(), 0),
    contact_force_estimate_(robot.numContacts(), Eigen::Vector3d::Zero()),
    contact_surface_normal_(robot.numContacts(), Eigen::Vector3d::Zero()),
    schmitt_trigger_(robot.numContacts(), 
                     SchmittTrigger(settings.schmitt_trigger_settings)),
    num_contacts_(robot.numContacts()) {
}


ContactEstimator::ContactEstimator() 
  : settings_(),
    contact_force_normal_estimate_(),
    contact_probability_(),
    contact_force_estimate_(),
    contact_surface_normal_(),
    schmitt_trigger_(),
    num_contacts_(0) {
}


ContactEstimator::~ContactEstimator() {
}


void ContactEstimator::reset() {
  for (int i=0; i<num_contacts_; ++i) {
    schmitt_trigger_[i].reset();
  }
}


void ContactEstimator::update(Robot& robot, const Eigen::VectorXd& tauJ, 
                              const std::vector<double>& force_sensor_raw) {
  // Estimate contact force from robot dynamics 
  for (int i=0; i<num_contacts_; ++i) {
    contact_force_estimate_[i].noalias() 
        = - robot.getJointContactJacobian(i).template block<3, 3>(0, i*3).transpose().inverse() 
            * (tauJ.template segment<3>(3*i)-robot.getJointInverseDynamics().template segment<3>(3*i));
    contact_force_normal_estimate_[i] = contact_force_estimate_[i].dot(contact_surface_normal_[i]);
  }
  // Contact probability 
  for (int i=0; i<num_contacts_; ++i) {
    contact_probability_ [i]
        = 1.0 / (1.0 + std::exp(- settings_.beta1 * contact_force_normal_estimate_[i]
                                - settings_.beta0));
    if (std::isnan(contact_probability_[i]) || std::isinf(contact_probability_[i])) {
      contact_probability_[i] = 0;
    }
  }
  // // Fuse force estimate and force sensor measurements
  // for (int i=0; i<num_contacts_; ++i) {
  //   schmitt_trigger_[i].update(force_sensor_raw[i]);
  // }
}


void ContactEstimator::setParameters(const ContactEstimatorSettings& settings) {
  settings_ = settings;
  for (auto& e : schmitt_trigger_) {
    e.setParameters(settings.schmitt_trigger_settings);
  }
}


std::vector<std::pair<int, bool>> ContactEstimator::getContactState(
    const double prob_threshold) const {
  std::vector<std::pair<int, bool>> contact_state;
  for (int i=0; i<num_contacts_; ++i) {
    contact_state.push_back(
        std::pair<int, bool>(i, (contact_probability_[i] >= prob_threshold)));
  }
  return contact_state;
}


const std::vector<Eigen::Vector3d>& ContactEstimator::getContactForceEstimate() const {
  return contact_force_estimate_;
}


const std::vector<double>& ContactEstimator::getContactForceNormalEstimate() const {
  return contact_force_normal_estimate_;
}


const std::vector<double>& ContactEstimator::getContactProbability() const {
  return contact_probability_;
}


const std::vector<double>& ContactEstimator::getForceSensorBias() const {
  return settings_.force_sensor_bias;
}


const std::vector<Eigen::Vector3d>& ContactEstimator::getContactSurfaceNormal() const {
  return contact_surface_normal_;
}


void ContactEstimator::setContactSurfaceNormal(
    const std::vector<Eigen::Vector3d>& contact_surface_normal) {
  contact_surface_normal_ = contact_surface_normal;
}


void ContactEstimator::setForceSensorBias(
    const std::vector<double>& force_sensor_bias) {
  settings_.force_sensor_bias = force_sensor_bias;
}

} // namespace legged_state_estimator
