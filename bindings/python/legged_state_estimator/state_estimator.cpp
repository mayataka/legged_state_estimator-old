#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "legged_state_estimator/state_estimator.hpp"
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

PYBIND11_MODULE(state_estimator, m) {
  py::class_<StateEstimator<double>>(m, "StateEstimator")
    .def(py::init<const StateEstimatorSettings<double>&>(),
          py::arg("state_estimator_settings"))
    .def(py::init<>())
    .def("update", &StateEstimator<double>::update,
          py::arg("imu_gyro_raw"), py::arg("imu_lin_accel_raw"), 
          py::arg("qJ"), py::arg("dqJ"), py::arg("tauJ"), py::arg("f"))
    .def("predict", &StateEstimator<double>::predict,
          py::arg("imu_gyro_raw"), py::arg("imu_lin_accel_raw"), 
          py::arg("qJ"), py::arg("dqJ"), py::arg("tauJ"), py::arg("f"),
          py::arg("lin_vel_pred"))
    .def_property_readonly("base_position_estimate", &StateEstimator<double>::getBasePositionEstimate)
    .def_property_readonly("base_orientation_estimate", &StateEstimator<double>::getBaseOrientationEstimate)
    .def_property_readonly("base_linear_velocity_estimate", &StateEstimator<double>::getBaseLinearVelocityEstimate)
    .def_property_readonly("imu_gyro_bias_estimate", &StateEstimator<double>::getIMUGyroBiasEstimate)
    .def_property_readonly("imu_linear_acceleration_bias_estimate", &StateEstimator<double>::getIMULinearAccelerationBiasEstimate)
    .def_property_readonly("base_angular_velocity_estimate", &StateEstimator<double>::getBaseAngularVelocityEstimate)
    .def_property_readonly("joint_velocity_estimate", &StateEstimator<double>::getJointVelocityEstimate)
    .def_property_readonly("joint_torque_estimate", &StateEstimator<double>::getJointTorqueEstimate);
}

} // namespace python
} // namespace legged_state_estimator