/*
 This class runs Python plugins for Gazebo.
 */

#include <iostream>
#include <stdexcept>
#include <ros/console.h>
#include "alpyca/sim/plugin_runner.h"

#include "altimeter_sensor_wrapper.h"
#include "contact_sensor_wrapper.h"
#include "depth_camera_sensor_wrapper.h"
#include "force_torque_sensor_wrapper.h"
#include "gps_sensor_wrapper.h"
#include "imu_sensor_wrapper.h"
#include "logical_camera_sensor_wrapper.h"
#include "magnetometer_sensor_wrapper.h"
#include "ray_sensor_wrapper.h"
#include "rfid_tag_wrapper.h"
#include "sonar_sensor_wrapper.h"
#include "wireless_receiver_wrapper.h"
#include "wireless_transceiver_wrapper.h"
#include "wireless_transmitter_wrapper.h"


using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(PluginRunner)

PluginRunner::PluginRunner() : SensorPlugin()
{
}

PluginRunner::~PluginRunner()
{
}

std::vector<std::string> split(const std::string& s, char delimiter)
{
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

std::string convert_upper_case(std::string s) 
{ 
    int n = s.length(); 
    int res_ind = 0; 
  
    for (int i = 0; i < n; i++) { 
        if (s[i] == '_') { 
            s[i + 1] = toupper(s[i + 1]); 
            continue; 
        } 
        else 
            s[res_ind++] = s[i];         
    } 
    s[res_ind] = '\0';
    s[0] = toupper(s[0]); 

    return s; 
} 

template<typename TWrapper, typename TSensor>
void PluginRunner::LoadTemplate(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  std::string python_plugin;
  if (_sdf->HasElement("python_plugin"))
    python_plugin = _sdf->Get<std::string>("python_plugin");
  else
  {
    throw std::invalid_argument("Parameter python_plugin does not exist!");
  }

  auto python_plugin_class = convert_upper_case(split(python_plugin, '.').back());

  sensor_wrapper = new TWrapper(std::dynamic_pointer_cast<TSensor>(_sensor));

  std::string sensor_name = _sensor->Type() + "Sensor";
  sensor_name[0] = toupper(sensor_name[0]);
  // Start the interpreter and keep it alive
  guard = new py::scoped_interpreter();
  
  // Import PyContactSensor to make type-casting to the pybind11 version of the contact sensor possible.
  py_sensor_module = py::module::import("sensors");
  py_msgs = py::module::import("msgs");
  py_sensor_class = py_sensor_module.attr(sensor_name.c_str());
  custom_plugin_module = py::module::import(python_plugin.c_str());
  plugin_class = custom_plugin_module.attr(python_plugin_class.c_str());

  plugin = plugin_class();
  load_func = plugin.attr("Load");
  load_func((TWrapper*)sensor_wrapper);//, _sdf);
}

void PluginRunner::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  /*
   Wrap the call of the Load function of the Python plugin.
  */

  std::string plugin_type = _sensor->Type();

  if(plugin_type == "altimeter")
  {
    LoadTemplate<AltimeterSensorWrapper, sensors::AltimeterSensor>(_sensor, _sdf);
  }
  else if(plugin_type == "contact")
  {
    LoadTemplate<ContactSensorWrapper, sensors::ContactSensor>(_sensor, _sdf);
  }
  else if(plugin_type == "depth_camera")
  {
    LoadTemplate<DepthCameraSensorWrapper, sensors::DepthCameraSensor>(_sensor, _sdf);
  }
  else if(plugin_type == "force_torque")
  {
    LoadTemplate<ForceTorqueSensorWrapper, sensors::ForceTorqueSensor>(_sensor, _sdf);
  }
  else if(plugin_type == "gps")
  {
    LoadTemplate<GpsSensorWrapper, sensors::GpsSensor>(_sensor, _sdf);
  }
  else if(plugin_type == "imu")
  {
    LoadTemplate<ImuSensorWrapper, sensors::ImuSensor>(_sensor, _sdf);
  }
  else if(plugin_type == "logical_camera")
  {
    LoadTemplate<LogicalCameraSensorWrapper, sensors::LogicalCameraSensor>(_sensor, _sdf);
  }
  else if(plugin_type == "magnetometer")
  {
    LoadTemplate<MagnetometerSensorWrapper, sensors::MagnetometerSensor>(_sensor, _sdf);
  }
  else if(plugin_type == "ray")
  {
    LoadTemplate<RaySensorWrapper, sensors::RaySensor>(_sensor, _sdf);
  }
  else if(plugin_type == "rfid_tag")
  {
    LoadTemplate<RFIDTagWrapper, sensors::RFIDTag>(_sensor, _sdf);
  }
  else if(plugin_type == "sonar")
  {
    LoadTemplate<SonarSensorWrapper, sensors::SonarSensor>(_sensor, _sdf);
  }
  else if(plugin_type == "wireless_receiver")
  {
    LoadTemplate<WirelessReceiverWrapper, sensors::WirelessReceiver>(_sensor, _sdf);
  }
  else if(plugin_type == "wireless_transceiver")
  {
    LoadTemplate<WirelessTransceiverWrapper, sensors::WirelessTransceiver>(_sensor, _sdf);
  }
  else if(plugin_type == "wireless_transmitter")
  {
    LoadTemplate<WirelessTransmitterWrapper, sensors::WirelessTransmitter>(_sensor, _sdf);
  }
  else
  {
    throw std::invalid_argument("Unknown base plugin!");
  }


}
