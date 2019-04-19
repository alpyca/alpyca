#ifndef _PLUGIN_RUNNER_
#define _PLUGIN_RUNNER_

#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include "sensor_wrapper.h"


namespace py = pybind11;

namespace gazebo
{

  class PluginRunner : public SensorPlugin
  {
    public: PluginRunner();
    public: virtual ~PluginRunner();

    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: void OnUpdate();

    private:
      py::object plugin;
      py::object py_sensor_class;
      py::object py_msgs;
      py::object plugin_class;
      py::object load_func;
      py::module py_sensor_module;
      py::module py_sys;
      py::module custom_plugin_module;
      SensorWrapper *sensor_wrapper;
      py::scoped_interpreter* guard;

      template<typename TWrapper, typename TSensor>
      void LoadTemplate(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
  };
}

#endif //_PLUGIN_RUNNER_
