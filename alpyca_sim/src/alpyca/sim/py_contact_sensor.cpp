/*
 Pybind11 wrappers for ContactSensorWrapper, Vector3d, ContactWrapper and ContactsWrapper.
 */
 
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include "alpyca/sim/contact_sensor_wrapper.h"


namespace py = pybind11;

namespace gazebo
{
    PYBIND11_MODULE(py_contact_sensor, m) {
        py::class_<ContactSensorWrapper>(m, "PyContactSensor")
        .def("Contacts", &ContactSensorWrapper::Contacts)
        .def("SetActive", &SensorWrapper::SetActive)
        .def("ConnectUpdated", &SensorWrapper::ConnectUpdated);
    }
}
