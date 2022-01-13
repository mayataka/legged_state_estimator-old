#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "legged_state_estimator/contact_estimator.hpp"


namespace legged_state_estimator {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(contact_estimator, m) {
  py::class_<ContactEstimator<double>>(m, "ContactEstimator")
    .def(py::init<const Eigen::Vector4d&, const int, const double, const double>(),
          py::arg("force_sensor_bias"), py::arg("window_filter_size"),
          py::arg("beta1"), py::arg("beta0"))
    .def(py::init<>())
    .def("reset", &ContactEstimator<double>::reset)
    .def("update", &ContactEstimator<double>::update,
          py::arg("f_raw"))
    .def("get_contact_force_estimate", &ContactEstimator<double>::getContactForceEstimate)
    .def("get_contact_probability", 
          static_cast<const Eigen::Vector4d& (ContactEstimator<double>::*)() const>(&ContactEstimator<double>::getContactProbability))
    .def("get_contact_probability", 
          static_cast<double (ContactEstimator<double>::*)(const int) const>(&ContactEstimator<double>::getContactProbability),
          py::arg("contact_id"))
    .def("get_non_contact_probability", &ContactEstimator<double>::getNonContactProbability)
    .def("get_force_bias", &ContactEstimator<double>::setForceBias,
          py::arg("force-bias"))
    .def("get_force_bias", &ContactEstimator<double>::getForceBias);
}

} // namespace python
} // namespace legged_state_estimator