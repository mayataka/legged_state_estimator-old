#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "legged_state_estimator/state_estimator_settings.hpp"


namespace legged_state_estimator {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(state_estimator_settings, m) {
  py::class_<StateEstimatorSettings>(m, "StateEstimatorSettings")
    .def(py::init<>())
    .def_static("a1_settings", &StateEstimatorSettings::A1Settings,
                 py::arg("path_to_urdf"), py::arg("dt"))
    .def_readwrite("path_to_urdf", &StateEstimatorSettings::path_to_urdf)
    .def_readwrite("contact_frames", &StateEstimatorSettings::contact_frames)
    .def_readwrite("contact_estimator_settings", &StateEstimatorSettings::contact_estimator_settings)
    .def_readwrite("inekf_noise_params", &StateEstimatorSettings::inekf_noise_params)
    .def_readwrite("contact_position_noise", &StateEstimatorSettings::contact_position_noise)
    .def_readwrite("contact_rotation_noise", &StateEstimatorSettings::contact_rotation_noise)
    .def_readwrite("dt", &StateEstimatorSettings::dt)
    .def_readwrite("lpf_gyro_accel_cutoff", &StateEstimatorSettings::lpf_gyro_accel_cutoff)
    .def_readwrite("lpf_lin_accel_cutoff", &StateEstimatorSettings::lpf_lin_accel_cutoff)
    .def_readwrite("lpf_dqJ_cutoff", &StateEstimatorSettings::lpf_dqJ_cutoff)
    .def_readwrite("lpf_ddqJ_cutoff", &StateEstimatorSettings::lpf_ddqJ_cutoff)
    .def_readwrite("lpf_tauJ_cutoff", &StateEstimatorSettings::lpf_dqJ_cutoff);
}

} // namespace python
} // namespace legged_state_estimator