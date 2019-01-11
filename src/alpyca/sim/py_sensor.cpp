#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include "alpyca/sim/sensor_wrapper.h"


namespace py = pybind11;

namespace gazebo
{
    PYBIND11_MODULE(py_sensor, m) {
        py::class_<SensorWrapper>(m, "PySensor")
        .def("SetActive", &SensorWrapper::SetActive)
        .def("ConnectUpdated", &SensorWrapper::ConnectUpdated);
    }
}
