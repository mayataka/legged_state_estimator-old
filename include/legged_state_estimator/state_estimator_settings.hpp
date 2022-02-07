#ifndef LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_SETTINGS_HPP_
#define LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_SETTINGS_HPP_

#include <string>
#include <vector>
#include <array>

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

template <typename Scalar>
struct StateEstimatorSettings {
public:
  using Vector4  = types::Vector4<Scalar>;

  std::string path_to_urdf;
  std::array<int, 4> contact_frames;

  Scalar dt;

  Scalar force_compl_cutoff_freq;
  Scalar beta1_logistic_reg; 
  Scalar beta0_logistic_reg;
  Vector4 force_sensor_bias;

  Scalar lpf_gyro_cutoff;
  Scalar lpf_dqJ_cutoff;
  Scalar lpf_tauJ_cutoff;

  Scalar cov_angular_vel;
  Scalar cov_linear_acc;
  Scalar cov_dqJ;

  static StateEstimatorSettings A1Settings(const std::string& path_to_urdf, 
                                           const Scalar dt) {
    StateEstimatorSettings settings;
    settings.path_to_urdf = path_to_urdf;
    settings.dt = dt;

    // unitree a1
    int FL_foot = 14;
    int FR_foot = 24;
    int RL_foot = 34;
    int RR_foot = 44;
    settings.contact_frames = {FL_foot, FR_foot, RL_foot, RR_foot};

    settings.force_compl_cutoff_freq = 100;
    settings.beta0_logistic_reg = -4.0;
    settings.beta1_logistic_reg = 0.25;
    settings.force_sensor_bias << 0.0, 0.0, 0.0, 0.0;

    settings.lpf_gyro_cutoff = 250;
    settings.lpf_dqJ_cutoff  = 250;
    settings.lpf_tauJ_cutoff = 250;

    settings.cov_angular_vel = 0.1;
    settings.cov_linear_acc  = 0.1;
    settings.cov_dqJ         = 0.001;

    return settings;
  }

};

} // namespace legged_state_estimator


#endif // LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_SETTINGS_HPP_ 