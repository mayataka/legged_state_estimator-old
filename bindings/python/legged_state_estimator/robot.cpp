#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "legged_state_estimator/robot.hpp"


namespace legged_state_estimator {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(robot, m) {
  py::enum_<pinocchio::ReferenceFrame>(m, "ReferenceFrame")
    .value("WORLD", pinocchio::ReferenceFrame::WORLD)
    .value("LOCAL", pinocchio::ReferenceFrame::LOCAL)
    .value("LOCAL_WORLD_ALIGNED", pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED)
    .export_values();

  py::class_<Robot>(m, "Robot")
    .def(py::init<const std::string&, const std::vector<int>&>(),
          py::arg("path_to_urdf"), py::arg("contact_frames"))
    .def(py::init<>())
    .def("update_leg_kinematics", &Robot::updateLegKinematics,
          py::arg("qJ"), py::arg("dqJ"), 
          py::arg("rf")=pinocchio::LOCAL_WORLD_ALIGNED)
    .def("update_kinematics", &Robot::updateKinematics,
          py::arg("base_pos"), py::arg("base_quat"), 
          py::arg("base_linear_vel"), py::arg("base_angular_vel"), 
          py::arg("qJ"), py::arg("dqJ"),
          py::arg("rf")=pinocchio::LOCAL_WORLD_ALIGNED)
    .def("update_leg_dynamics", &Robot::updateLegDynamics,
          py::arg("qJ"), py::arg("dqJ"), py::arg("ddqJ"))
    .def("update_dynamics", &Robot::updateDynamics,
          py::arg("base_pos"), py::arg("base_quat"), 
          py::arg("base_linear_vel"), py::arg("base_angular_vel"), 
          py::arg("base_linear_acc"), py::arg("base_angular_acc"), 
          py::arg("qJ"), py::arg("dqJ"), py::arg("ddqJ"))
    .def("get_base_position", &Robot::getBasePosition)
    .def("get_base_rotation", &Robot::getBaseRotation)
    .def("get_contact_position", &Robot::getContactPosition,
          py::arg("contact_id"))
    .def("get_contact_rotation", &Robot::getContactRotation,
          py::arg("contact_id"))
    .def("get_contact_jacobian", &Robot::getContactJacobian,
          py::arg("contact_id"))
    .def("get_joint_contact_jacobian", &Robot::getJointContactJacobian,
          py::arg("contact_id"))
    .def("get_inverse_dynamics", &Robot::getInverseDynamics)
    .def("get_joint_inverse_dynamics", &Robot::getJointInverseDynamics);
}

} // namespace python
} // namespace legged_state_estimator