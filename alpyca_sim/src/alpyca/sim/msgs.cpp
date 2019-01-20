
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <gazebo/gazebo.hh>

#include "alpyca/sim/repeated.h"


namespace py = pybind11;

namespace gazebo
{
    namespace msgs
    {
        PYBIND11_MODULE(msgs, m) {
            py::class_<Repeated<double>>(m, "Repeated_double")
            .def("__getitem__", &Repeated<double>::getitem);


            py::class_<Vector3d>(m, "Vector3d")
            .def_property_readonly("x", &Vector3d::x)
            .def_property_readonly("y", &Vector3d::y)
            .def_property_readonly("z", &Vector3d::z);

            py::class_<Repeated<Vector3d>>(m, "Repeated_Vector3d")
            .def("__getitem__", &Repeated<Vector3d>::getitem);


            py::class_<Contact>(m, "Contact")
            .def_property_readonly("collision1", &Contact::collision1)
            .def_property_readonly("collision2", &Contact::collision2)
            .def_property_readonly("depth", [](Contact *obj) -> Repeated<double>
            {
                std::function<double(int)> func = [obj](int index) { return obj->depth(index); };
                return Repeated<double>(func, obj->depth_size());
            })
            .def_property_readonly("position", [](Contact *obj) -> Repeated<Vector3d>
            {
                std::function<Vector3d(int)> func = [obj](int index) { return obj->position(index); };
                return Repeated<Vector3d>(func, obj->position_size());
            })
            .def_property_readonly("normal", [](Contact *obj) -> Repeated<Vector3d>
            {
                std::function<Vector3d(int)> func = [obj](int index) { return obj->normal(index); };
                return Repeated<Vector3d>(func, obj->normal_size());
            });

            py::class_<Repeated<Contact>>(m, "Repeated_Contact")
            .def("__getitem__", &Repeated<Contact>::getitem);


            py::class_<Contacts>(m, "Contacts")
            .def_property_readonly("contact", [](Contacts *obj) -> Repeated<Contact>
            {
                std::function<Contact(int)> func = [obj](int index) { return obj->contact(index); };
                return Repeated<Contact>(func, obj->contact_size());
            });
        }
    }
}
