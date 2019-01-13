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

        py::class_<gazebo::msgs::Vector3d>(m, "PyVector3d")
        .def_property_readonly("x", &gazebo::msgs::Vector3d::x)
        .def_property_readonly("y", &gazebo::msgs::Vector3d::y)
        .def_property_readonly("z", &gazebo::msgs::Vector3d::z);

        py::class_<msgs::Contact>(m, "PyContact")
        .def_property_readonly("collision1", &msgs::Contact::collision1)
        .def_property_readonly("collision2", &msgs::Contact::collision2)
        .def_property_readonly("depth", [](msgs::Contact *contact) -> std::vector<double>
        {
            std::vector<double> dep = {};
            for (unsigned int i = 0; i < contact->position_size(); ++i)
            {
                dep.push_back(contact->depth(i));
            }

            return dep;
        })
        .def_property_readonly("position", [](msgs::Contact *contact) -> std::vector<gazebo::msgs::Vector3d>
        {
            std::vector<gazebo::msgs::Vector3d> pos = {};
            for (unsigned int i = 0; i < contact->position_size(); ++i)
            {
                pos.push_back(contact->position(i));
            }

            return pos;
        })
        .def_property_readonly("normal", [](msgs::Contact *contact) -> std::vector<gazebo::msgs::Vector3d>
        {
            std::vector<gazebo::msgs::Vector3d> norm = {};
            for (unsigned int i = 0; i < contact->position_size(); ++i)
            {
                norm.push_back(contact->normal(i));
            }

            return norm;
        });

        py::class_<msgs::Contacts>(m, "PyContacts")
        .def("__getitem__", [](msgs::Contacts *contacts, int index) -> msgs::Contact
        {
            if(index >= contacts->contact_size())
            {
                throw py::index_error();
            }

            return contacts->contact(index);
        });
    }
}
