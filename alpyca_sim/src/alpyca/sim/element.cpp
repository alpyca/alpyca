/*
 Pybind11 wrappers for a SDF element
 */
 
#include <pybind11/pybind11.h>
#include <gazebo/gazebo.hh>

namespace py = pybind11;

namespace gazebo
{
    PYBIND11_MODULE(element, m) {
        py::class_<sdf::Element, std::shared_ptr<sdf::Element>>(m, "Element")
        .def("HasElement", &sdf::Element::HasElement)
        .def("GetElement", py::overload_cast<const std::string &>(&sdf::Element::GetElement))
        .def("GetString", &sdf::Element::Get<std::string>)
        .def("GetInt", &sdf::Element::Get<int>)
        .def("GetDouble", &sdf::Element::Get<double>)
        .def("GetBool", &sdf::Element::Get<bool>);
    }
}
