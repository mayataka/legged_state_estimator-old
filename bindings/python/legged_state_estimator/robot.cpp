#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "legged_state_estimator/robot.hpp"
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

PYBIND11_MODULE(robot, m) {
  py::class_<Robot<double>>(m, "Robot")
    .def(py::init<const std::string&, const std::array<int, 4>&>(),
          py::arg("path_to_urdf"), py::arg("contact_frames"))
    .def(py::init<>())
    .def("update_base_configuration", &Robot<double>::updateBaseConfiguration,
          py::arg("base_pos"), py::arg("base_quat"), 
          py::arg("base_linear_vel"), py::arg("base_angular_vel"), 
          py::arg("dt")=1.0)
    .def("update_base_kinematics", &Robot<double>::updateBaseKinematics,
          py::arg("base_pos"), py::arg("base_quat"), 
          py::arg("base_linear_vel"), py::arg("base_angular_vel"), 
          py::arg("base_linear_acc"), py::arg("dt")=1.0)
    .def("update_leg_kinematics", [](Robot<double>& self, 
                                     const Vector3d& base_angular_vel,
                                     const Vector12d& qJ, const Vector12d& dqJ) {
      self.updateLegKinematics(base_angular_vel, qJ, dqJ);
      }, py::arg("base_angular_vel"), py::arg("qJ"), py::arg("dqJ"))
    .def("update_kinematics", [](Robot<double>& self, 
                                 const Vector3d& base_pos, const Vector4d& base_quat,
                                 const Vector3d& base_linear_vel,
                                 const Vector3d& base_angular_vel,
                                 const Vector12d& qJ, const Vector12d& dqJ) {
      self.updateKinematics(base_pos, base_quat,  
                            base_linear_vel, base_angular_vel, qJ, dqJ);
      }, py::arg("base_pos"), py::arg("base_quat"), 
         py::arg("base_linear_vel"), py::arg("base_angular_vel"), 
         py::arg("qJ"), py::arg("dqJ"))
    .def("update_leg_dynamics", &Robot<double>::updateLegDynamics,
          py::arg("qJ"), py::arg("dqJ"),
          py::arg("base_linear_acc")=Vector3d::Zero(), 
          py::arg("base_angular_acc")=Vector3d::Zero())
    .def("update_dynamics", &Robot<double>::updateDynamics,
          py::arg("base_pos"), py::arg("base_quat"), 
          py::arg("base_linear_vel"), py::arg("base_angular_vel"), 
          py::arg("qJ"), py::arg("dqJ"),
          py::arg("base_linear_acc")=Vector3d::Zero(), 
          py::arg("base_angular_acc")=Vector3d::Zero())
    .def_property_readonly("base_position", &Robot<double>::getBasePosition)
    .def_property_readonly("base_orientation", &Robot<double>::getBaseOrientation)
    .def_property_readonly("base_linear_velocity", &Robot<double>::getBaseLinearVelocity)
    .def_property_readonly("base_jacobian_wrt_configuration", &Robot<double>::getBaseJacobianWrtConfiguration)
    .def_property_readonly("base_jacobian_wrt_velocity", &Robot<double>::getBaseJacobianWrtVelocity)
    .def("contact_position", &Robot<double>::getContactPosition,
         py::arg("contact_index"))
    .def("contact_velocity", &Robot<double>::getContactVelocity,
         py::arg("contact_index"))
    .def("contact_jacobian", &Robot<double>::getContactJacobian,
         py::arg("contact_index"))
    .def_property_readonly("dynamics", &Robot<double>::getDynamics)
    .def_property_readonly("joint_dynamics", &Robot<double>::getJointDynamics)
    .def_property_readonly("contact_frames", &Robot<double>::getContactFrames);
}

} // namespace python
} // namespace legged_state_estimator