#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "legged_state_estimator/contact_estimator.hpp"


namespace legged_state_estimator {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(contact_estimator, m) {
  py::class_<ContactEstimatorSettings>(m, "ContactEstimatorSettings")
    .def(py::init<>())
    .def_readwrite("beta0", &ContactEstimatorSettings::beta0)
    .def_readwrite("beta1", &ContactEstimatorSettings::beta1)
    .def_readwrite("force_sensor_bias", &ContactEstimatorSettings::force_sensor_bias)
    .def_readwrite("schmitt_trigger_settings", &ContactEstimatorSettings::schmitt_trigger_settings);

  py::class_<ContactEstimator>(m, "ContactEstimator")
    .def(py::init<const Robot&, const ContactEstimatorSettings&>(),
          py::arg("robot"), py::arg("settings"))
    .def(py::init<>())
    .def("reset", &ContactEstimator::reset)
    .def("update", &ContactEstimator::update,
          py::arg("robot"), py::arg("tauJ"), py::arg("force_sensor_raw"))
    .def("get_contact_state", &ContactEstimator::getContactState,
          py::arg("prob_threshold")=0.5)
    .def("get_contact_force_estimate", &ContactEstimator::getContactForceEstimate)
    .def("get_contact_force_normal_estimate", &ContactEstimator::getContactForceNormalEstimate)
    .def("get_contact_probability", &ContactEstimator::getContactProbability)
    .def("get_force_sensor_bias", &ContactEstimator::getForceSensorBias)
    .def("get_contact_surface_normal", &ContactEstimator::getContactSurfaceNormal)
    .def("set_force_sensor_bias", &ContactEstimator::setForceSensorBias,
          py::arg("force_sensor_bias"))
    .def("set_contact_surface_normal", &ContactEstimator::setContactSurfaceNormal,
          py::arg("contact_surface_normal"));
}

} // namespace python
} // namespace legged_state_estimator