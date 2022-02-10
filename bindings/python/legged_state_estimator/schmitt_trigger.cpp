#include <pybind11/pybind11.h>

#include "legged_state_estimator/schmitt_trigger.hpp"


namespace legged_state_estimator {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(schmitt_trigger, m) {
  py::class_<SchmittTriggerSettings>(m, "SchmittTriggerSettings")
    .def(py::init<>())
    .def_readwrite("lower_threshold", &SchmittTriggerSettings::lower_threshold)
    .def_readwrite("higher_threshold", &SchmittTriggerSettings::higher_threshold)
    .def_readwrite("lower_time_delay", &SchmittTriggerSettings::lower_time_delay)
    .def_readwrite("higher_time_delay", &SchmittTriggerSettings::higher_time_delay);

  py::class_<SchmittTrigger>(m, "SchmittTrigger")
    .def(py::init<const SchmittTriggerSettings&>(),
          py::arg("settings"))
    .def(py::init<>())
    .def("reset", &SchmittTrigger::reset)
    .def("update", &SchmittTrigger::update,
          py::arg("current_time"), py::arg("value"))
    .def("get_state", &SchmittTrigger::getState)
    .def("set_parameters", &SchmittTrigger::setParameters,
          py::arg("settings"));
}

} // namespace python
} // namespace legged_state_estimator