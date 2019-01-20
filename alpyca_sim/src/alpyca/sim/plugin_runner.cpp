/*
 This class runs Python plugins for Gazebo.
 */

#include <iostream>
#include <ros/console.h>
#include "alpyca/sim/plugin_runner.h"


using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(PluginRunner)

PluginRunner::PluginRunner() : SensorPlugin()
{
}

PluginRunner::~PluginRunner()
{
}

void PluginRunner::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  /*
   Wrap the call of the Load function of the Python plugin.
  */
  sensor_wrapper = new ContactSensorWrapper(_sensor);
  // Start the interpreter and keep it alive
  guard = new py::scoped_interpreter();
  
  // Import PyContactSensor to make type-casting to the pybind11 version of the contact sensor possible.
  py_sensor_module = py::module::import("py_contact_sensor");
  py_msgs = py::module::import("msgs");
  py_sensor_class = py_sensor_module.attr("PyContactSensor");
  custom_plugin_module = py::module::import("alpyca.sim.contact_plugin");
  plugin_class = custom_plugin_module.attr("ContactPlugin");

  plugin = plugin_class();
  load_func = plugin.attr("Load");
  load_func(sensor_wrapper);//, _sdf);
}
