#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "legged_state_estimator/leg_odometry.hpp"


namespace legged_state_estimator {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(leg_odometry, m) {
  py::class_<LegOdometry<double>>(m, "LegOdometry")
    .def(py::init<const std::string&, const std::array<int, 4>&, 
                  const double, const double>(),
          py::arg("path_to_urdf"), py::arg("contact_frames"), 
          py::arg("dt"), py::arg("cf_cutoff_freq"))
    .def(py::init<>())
    .def("update_base_state_estimation", &LegOdometry<double>::updateBaseStateEstimation,
          py::arg("quat"), py::arg("gyro"), py::arg("qJ"), py::arg("dqJ"), 
          py::arg("contact_probability"))
    .def("update_contact_position_estimation", &LegOdometry<double>::updateContactPositionEstimation,
          py::arg("base_lin_vel_est"), py::arg("contact_probability"))
    .def("reset_base_state_estimation", &LegOdometry<double>::resetBaseStateEstimation,
          py::arg("base_x")=0, py::arg("base_y")=0, 
          py::arg("contact_height")=Eigen::Vector4d::Zero())
    .def("reset_contact_position_estimation", &LegOdometry<double>::resetContactPositionEstimation,
          py::arg("base_x")=0, py::arg("base_y")=0, 
          py::arg("contact_height")=Eigen::Vector4d::Zero())
    .def("get_base_linear_velocity_estimate", &LegOdometry<double>::getBaseLinearVelocityEstimate)
    .def("get_base_position_estimate", &LegOdometry<double>::getBasePositionEstimate);
}

} // namespace python
} // namespace legged_state_estimator