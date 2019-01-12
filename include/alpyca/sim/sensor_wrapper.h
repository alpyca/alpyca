/*
 Wrapper for Gazebo Sensor.	
 */


#ifndef _SENSOR_WRAPPER_
#define _SENSOR_WRAPPER_

#include <functional>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <pybind11/pybind11.h>


namespace py = pybind11;

namespace gazebo
{
    class SensorWrapper{
    public:
        SensorWrapper(sensors::SensorPtr _sensor) : sensor(_sensor)
        {
        }

        void SetActive(const bool value)
        {
            sensor->SetActive(value);
        }

        void ConnectUpdated(std::function< void()> &_subscriber)
        {
            subscriber = _subscriber;
            updateConnection = sensor->ConnectUpdated(std::bind(&SensorWrapper::OnUpdate, this));
        }

        void OnUpdate()
        {
		    // The gil has to be acquired by this thread to make the Python function not block.
            py::gil_scoped_release release;
            {
                py::gil_scoped_acquire acquire;
                subscriber();
            }
        }

    private:
        sensors::SensorPtr sensor;
        event::ConnectionPtr updateConnection;
        std::function< void()> subscriber;
    };
}

#endif //_SENSOR_WRAPPER_
