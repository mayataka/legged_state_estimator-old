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
    .def_readwrite("force_compl_cutoff_freq", &StateEstimatorSettings<double>::force_compl_cutoff_freq)
    .def_readwrite("beta1_logistic_reg", &StateEstimatorSettings<double>::beta1_logistic_reg)
    .def_readwrite("beta0_logistic_reg", &StateEstimatorSettings<double>::beta0_logistic_reg)
    .def_readwrite("force_sensor_bias", &StateEstimatorSettings<double>::force_sensor_bias)
    .def_readwrite("lpf_gyro_cutoff", &StateEstimatorSettings<double>::lpf_gyro_cutoff)
    .def_readwrite("lpf_dqJ_cutoff", &StateEstimatorSettings<double>::lpf_dqJ_cutoff)
    .def_readwrite("lpf_tauJ_cutoff", &StateEstimatorSettings<double>::lpf_dqJ_cutoff);
}

} // namespace python
} // namespace legged_state_estimator