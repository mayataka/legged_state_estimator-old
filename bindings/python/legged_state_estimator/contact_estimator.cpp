#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "legged_state_estimator/contact_estimator.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {
namespace python {

namespace py = pybind11;

using Vector19d = types::Vector19<double>;
using Vector18d = types::Vector18<double>;
using Vector16d = types::Vector16<double>;
using Vector12d = types::Vector12<double>;
using Vector4d = types::Vector4<double>;
using Vector3d = types::Vector3<double>;
using Quaterniond = types::Quaternion<double>;
using Jacobian6Dd = types::Matrix<double, 6, 18>;

PYBIND11_MODULE(contact_estimator, m) {
  py::class_<ContactEstimator<double>>(m, "ContactEstimator")
    .def(py::init<const double, const double, const double, const double, const Eigen::Vector4d&>(),
          py::arg("sampling_time"), py::arg("compl_cutoff_freq"),
          py::arg("beta1"), py::arg("beta0"), py::arg("force_sensor_bias"))
    .def(py::init<>())
    .def("update", &ContactEstimator<double>::update,
          py::arg("robot"), py::arg("tauJ"), py::arg("force_sensor_raw"))
    .def("set_contact_surface_normal", 
          static_cast<void (ContactEstimator<double>::*)(const std::array<Vector3d, 4>&)>(&ContactEstimator<double>::setContactSurfaceNormal),
          py::arg("contact_surface_normal"))
    .def("set_contact_surface_normal", 
          static_cast<void (ContactEstimator<double>::*)(const Vector3d&, const int)>(&ContactEstimator<double>::setContactSurfaceNormal),
          py::arg("contact_surface_normal"), py::arg("contact_index"))
    .def("get_contact_force_estimate", 
          static_cast<const std::array<Vector3d, 4>& (ContactEstimator<double>::*)() const>(&ContactEstimator<double>::getContactForceEstimate))
    .def("get_contact_force_estimate", 
          static_cast<const Vector3d& (ContactEstimator<double>::*)(const int) const>(&ContactEstimator<double>::getContactForceEstimate),
          py::arg("contact_index"));
}

} // namespace python
} // namespace legged_state_estimator