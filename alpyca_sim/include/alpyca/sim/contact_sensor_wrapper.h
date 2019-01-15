/*
 Wrapper for Gazebo Contact and Contacts. "Repeated" proto message are converted to vector.	
 */

#ifndef _CONTACT_SENSOR_WRAPPER_H_
#define _CONTACT_SENSOR_WRAPPER_H_


#include <functional>
#include <iostream>
#include <vector>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <pybind11/pybind11.h>
#include "alpyca/sim/sensor_wrapper.h"


namespace py = pybind11;

namespace gazebo
{
    class ContactSensorWrapper : public SensorWrapper
    {
    public:
        ContactSensorWrapper(sensors::SensorPtr _sensor) : SensorWrapper(_sensor)
        {
            parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
        }

        msgs::Contacts Contacts()
        {
            return parentSensor->Contacts();
        }

    private:
        sensors::ContactSensorPtr parentSensor;
    };
}

#endif //_CONTACT_SENSOR_WRAPPER_H_
