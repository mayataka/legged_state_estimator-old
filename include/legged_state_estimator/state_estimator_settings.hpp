#ifndef LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_SETTINGS_HPP_
#define LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_SETTINGS_HPP_

#include <string>
#include <vector>

#include "inekf/NoiseParams.h"

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/contact_estimator.hpp"


namespace legged_state_estimator {

struct StateEstimatorSettings {
public:
  std::string path_to_urdf;
  std::vector<int> contact_frames;

  ContactEstimatorSettings contact_estimator_settings;

  inekf::NoiseParams inekf_noise_params;

  double contact_position_noise;
  double contact_rotation_noise;

  double dt;

  double lpf_gyro_cutoff;
  double lpf_gyro_accel_cutoff;
  double lpf_lin_accel_cutoff;
  double lpf_dqJ_cutoff;
  double lpf_ddqJ_cutoff;
  double lpf_tauJ_cutoff;


  static StateEstimatorSettings A1Settings(const std::string& path_to_urdf, 
                                           const double dt) {
    StateEstimatorSettings settings;
    settings.path_to_urdf = path_to_urdf;
    settings.contact_frames = {14, 24, 34, 44};

    settings.contact_estimator_settings.beta0 = -4.0;
    settings.contact_estimator_settings.beta1 =  0.25;
    settings.contact_estimator_settings.force_sensor_bias = {0.0, 0.0, 0.0, 0.0};
    settings.contact_estimator_settings.schmitt_trigger_settings.lower_threshold   = 0;
    settings.contact_estimator_settings.schmitt_trigger_settings.higher_threshold  = 0;
    settings.contact_estimator_settings.schmitt_trigger_settings.lower_time_delay  = 0;
    settings.contact_estimator_settings.schmitt_trigger_settings.higher_time_delay = 0;

    settings.inekf_noise_params.setGyroscopeNoise(0.01);
    settings.inekf_noise_params.setAccelerometerNoise(0.1);
    settings.inekf_noise_params.setGyroscopeBiasNoise(0.00001);
    settings.inekf_noise_params.setAccelerometerBiasNoise(0.0001);
    settings.inekf_noise_params.setContactNoise(0.1);

    settings.contact_position_noise = 0.01;
    settings.contact_rotation_noise = 0.01;

    settings.dt = dt;

    settings.lpf_gyro_cutoff       = 250;
    settings.lpf_gyro_accel_cutoff = 250;
    settings.lpf_lin_accel_cutoff  = 250;
    settings.lpf_dqJ_cutoff        = 250;
    settings.lpf_ddqJ_cutoff       = 250;
    settings.lpf_tauJ_cutoff       = 250;

    return settings;
  }

};

} // namespace legged_state_estimator


#endif // LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_SETTINGS_HPP_ 