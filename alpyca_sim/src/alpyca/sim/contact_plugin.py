#!/usr/bin/env python

import std_msgs
import rospy

class ContactPlugin:
    """ A Gazebo ContactPlugin written completely in Python """

    def __init__(self):
        self.sensor = None

    def Load(self, sensor, sdf):
        """ Load the sensor plugin
        
        Parameters
        ----------
        sensor : PySensor
            The sensor that loaded this plugin
        """

        self.sensor = sensor

        rospy.init_node('talker', anonymous=True)

        self.pub = rospy.Publisher('text', std_msgs.msg.String, queue_size=10)
        if sdf.HasElement('text'):
            self.text = sdf.GetString('text')
            print('text: ' + self.text)

        sensor.ConnectUpdated(self.OnUpdate)
        sensor.SetActive(True)

    def OnUpdate(self):
        """ Everytime the sensor is updated, this function is called.
		It receives the sensor's update signal.
        """
      
        contacts = self.sensor.Contacts()
        for contact in contacts.contact: # why does self.sensor.Contacts.contact not work?
            print("Collision between[{}] and [{}]".format(contact.collision1, contact.collision2))

            for (position, normal, depth) in zip(contact.position, contact.normal, contact.depth):
                print("Position: {} {} {}".format(position.x, position.y, position.z))
                print("Normal: {} {} {}".format(normal.x, normal.y, normal.z))
                print("Depth: {}".format(depth))

        self.pub.publish(std_msgs.msg.String(self.text))
