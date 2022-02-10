#ifndef LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_SETTINGS_HPP_
#define LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_SETTINGS_HPP_

#include <string>
#include <vector>

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/contact_estimator.hpp"


namespace legged_state_estimator {

struct StateEstimatorSettings {
public:
  std::string path_to_urdf;
  std::vector<int> contact_frames;

  ContactEstimatorSettings contact_estimator_settings;

  inekf::NoiseParams inekf_noise_params;

  double dt;

  double lpf_gyro_cutoff;
  double lpf_dqJ_cutoff;
  double lpf_tauJ_cutoff;


  static StateEstimatorSettings A1Settings(const std::string& path_to_urdf, 
                                           const double dt) {
    StateEstimatorSettings settings;
    settings.path_to_urdf = path_to_urdf;
    settings.contact_frames = {14, 24, 34, 44};

    settings.contact_estimator_settings.beta0 = -4.0;
    settings.contact_estimator_settings.beta1 =  0.25;
    settings.contact_estimator_settings.force_sensor_bias = {0.0, 0.0, 0.0, 0.0};

    settings.inekf_noise_params.setGyroscopeNoise(0.01);
    settings.inekf_noise_params.setAccelerometerNoise(0.1);
    settings.inekf_noise_params.setGyroscopeBiasNoise(0.00001);
    settings.inekf_noise_params.setAccelerometerBiasNoise(0.0001);
    settings.inekf_noise_params.setContactNoise(0.1);

    settings.dt = dt;

    settings.lpf_gyro_cutoff = 250;
    settings.lpf_dqJ_cutoff  = 250;
    settings.lpf_tauJ_cutoff = 250;

    return settings;
  }

};

} // namespace legged_state_estimator


#endif // LEGGED_STATE_ESTIMATOR_STATE_ESTIMATOR_SETTINGS_HPP_ 