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
    class ContactWrapper
    {
    public:
        ContactWrapper(msgs::Contact _contact) : contact(_contact)
        {
        }

        std::vector<double> depth()
        {
            std::vector<double> dep = {};
            for (unsigned int i = 0; i < contact.position_size(); ++i)
            {
                dep.push_back(contact.depth(i));
            }

            return dep;
        }

        const::std::string & collision1()
        {
            return contact.collision1();
        }
        
        const::std::string & collision2()
        {
            return contact.collision2();
        }
       
        std::vector<gazebo::msgs::Vector3d> position()
        {
            std::vector<gazebo::msgs::Vector3d> pos = {};
            for (unsigned int i = 0; i < contact.position_size(); ++i)
            {
                pos.push_back(contact.position(i));
            }

            return pos;
        }

        std::vector<gazebo::msgs::Vector3d> normal()
        {
            std::vector<gazebo::msgs::Vector3d> norm = {};
            for (unsigned int i = 0; i < contact.position_size(); ++i)
            {
                norm.push_back(contact.normal(i));
            }

            return norm;
        }

    private:
        msgs::Contact contact;
    };

    class ContactsWrapper
    {
    public:
        ContactsWrapper(msgs::Contacts _contacts) : contacts(_contacts)
        {
        }

        ContactWrapper contact(int index)
        {
            if(index >= contacts.contact_size())
            {
                throw py::index_error();
            }

            return contacts.contact(index);           
        }

    private:
        msgs::Contacts contacts;
    };

    class ContactSensorWrapper : public SensorWrapper
    {
    public:
        ContactSensorWrapper(sensors::SensorPtr _sensor) : SensorWrapper(_sensor)
        {
            parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
        }

        ContactsWrapper Contacts()
        {
            return ContactsWrapper(parentSensor->Contacts());
        }

    private:
        sensors::ContactSensorPtr parentSensor;
    };
}

#endif //_CONTACT_SENSOR_WRAPPER_H_
