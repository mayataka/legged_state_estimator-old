#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "legged_state_estimator/state_estimator_settings.hpp"


namespace legged_state_estimator {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(state_estimator_settings, m) {
  py::class_<StateEstimatorSettings<double>>(m, "StateEstimatorSettings")
    .def(py::init(&StateEstimatorSettings<double>::defaultSettings),
          py::arg("path_to_urdf"), py::arg("dt"))
    .def(py::init<>())
    .def_readwrite("path_to_urdf", &StateEstimatorSettings<double>::path_to_urdf)
    .def_readwrite("contact_frames", &StateEstimatorSettings<double>::contact_frames)
    .def_readwrite("dt", &StateEstimatorSettings<double>::dt)
    .def_readwrite("force_sensor_bias", &StateEstimatorSettings<double>::force_sensor_bias)
    .def_readwrite("contact_window_filter_size", &StateEstimatorSettings<double>::contact_window_filter_size)
    .def_readwrite("contact_probability_beta0", &StateEstimatorSettings<double>::contact_probability_beta0)
    .def_readwrite("contact_probability_beta1", &StateEstimatorSettings<double>::contact_probability_beta1)
    .def_readwrite("lpf_dqJ_cutoff", &StateEstimatorSettings<double>::lpf_dqJ_cutoff)
    .def_readwrite("lpf_gyro_cutoff", &StateEstimatorSettings<double>::lpf_gyro_cutoff)
    .def_readwrite("hpf_contact_frame_pos_cutoff", &StateEstimatorSettings<double>::hpf_contact_frame_pos_cutoff)
    .def_readwrite("cf_base_lin_vel_cutoff", &StateEstimatorSettings<double>::cf_base_lin_vel_cutoff)
    .def_readwrite("cf_base_pos_cutoff", &StateEstimatorSettings<double>::cf_base_pos_cutoff);
}

} // namespace python
} // namespace legged_state_estimator