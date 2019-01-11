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

        py::class_<gazebo::msgs::Vector3d>(m, "PyVector3d")
        .def_property_readonly("x", &gazebo::msgs::Vector3d::x)
        .def_property_readonly("y", &gazebo::msgs::Vector3d::y)
        .def_property_readonly("z", &gazebo::msgs::Vector3d::z);

        py::class_<ContactWrapper>(m, "PyContact")
        .def_property_readonly("collision1", &ContactWrapper::collision1)
        .def_property_readonly("collision2", &ContactWrapper::collision2)
        .def_property_readonly("position", &ContactWrapper::position)
        .def_property_readonly("normal", &ContactWrapper::normal)
        .def_property_readonly("depth", &ContactWrapper::depth);

        py::class_<ContactsWrapper>(m, "PyContacts")
        .def("__getitem__", &ContactsWrapper::contact);
    }
}
