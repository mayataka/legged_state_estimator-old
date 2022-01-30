#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "legged_state_estimator/state_estimator.hpp"


namespace legged_state_estimator {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(state_estimator, m) {
  py::class_<StateEstimator<double>>(m, "StateEstimator")
    .def(py::init<const StateEstimatorSettings<double>&>(),
          py::arg("state_estimator_settings"))
    .def(py::init<>())
    .def("add_calibration_data", &StateEstimator<double>::addCalibrationData,
          py::arg("gyro_raw"), py::arg("accel_raw"))
    .def("do-calibration", &StateEstimator<double>::doCalibration,
          py::arg("quat"))
    .def("update", &StateEstimator<double>::update,
          py::arg("quat"), py::arg("imu_gyro_raw"), py::arg("imu_lin_accel_raw"), 
          py::arg("qJ"), py::arg("dqJ"), py::arg("f"))
//     .def("predict", &StateEstimator<double>::predict,
//           py::arg("quat"), py::arg("imu_gyro_raw"), py::arg("base_accel_pred"), 
//           py::arg("qJ"), py::arg("dqJ"), py::arg("f"))
    .def("reset", &StateEstimator<double>::reset,
          py::arg("base_x")=0, py::arg("base_y")=0, 
          py::arg("contact_height")=Eigen::Vector4d::Zero())
    .def("get_base_linear_velocity_estimate", &StateEstimator<double>::getBaseLinearVelocityEstimate)
    .def("get_base_angular_velocity_estimate", &StateEstimator<double>::getBaseAngularVelocityEstimate)
    .def("get_base_position_estimate", &StateEstimator<double>::getBasePositionEstimate)
    .def("get_joint_velocity_estimate", &StateEstimator<double>::getJointVelocityEstimate)
    .def("get_contact_force_estimate", &StateEstimator<double>::getContactForceEstimate)
    .def("get_contact_probability", 
          static_cast<const Eigen::Vector4d& (StateEstimator<double>::*)() const>(&StateEstimator<double>::getContactProbability))
    .def("get_contact_probability", 
          static_cast<double (StateEstimator<double>::*)(const int) const>(&StateEstimator<double>::getContactProbability),
          py::arg("contact_id"))
    .def("get_non_contact_probability", &StateEstimator<double>::getNonContactProbability)
    .def("get_contact_frame_position_estimate", 
          static_cast<const std::array<Eigen::Vector3d, 4>& (StateEstimator<double>::*)() const>(&StateEstimator<double>::getContactFramePositionEstimate))
    .def("get_contact_frame_position_estimate", 
          static_cast<const Eigen::Vector3d& (StateEstimator<double>::*)(const int) const>(&StateEstimator<double>::getContactFramePositionEstimate),
          py::arg("contact_id"))
    .def("get_contact_frame_velocity_estimate", 
          static_cast<const std::array<Eigen::Vector3d, 4>& (StateEstimator<double>::*)() const>(&StateEstimator<double>::getContactFrameVelocityEstimate))
    .def("get_contact_frame_velocity_estimate", 
          static_cast<const Eigen::Vector3d& (StateEstimator<double>::*)(const int) const>(&StateEstimator<double>::getContactFrameVelocityEstimate),
          py::arg("contact_id"));
}

} // namespace python
} // namespace legged_state_estimator