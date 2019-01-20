#!/usr/bin/env python


class ContactPlugin:
    """ A Gazebo ContactPlugin written completely in Python """

    def __init__(self):
        self.sensor = None

    def Load(self, sensor):
        """ Load the sensor plugin
        
        Parameters
        ----------
        sensor : PySensor
            The sensor that loaded this plugin
        """

        self.sensor = sensor

        sensor.ConnectUpdated(self.OnUpdate)
        sensor.SetActive(True)

    def OnUpdate(self):
        """ Everytime the sensor is updated, this function is called.
		It receives the sensor's update signal.
        """
      
        contacts = self.sensor.Contacts
        for contact in contacts.contact: # why does self.sensor.Contacts.contact not work?
            print("Collision between[{}] and [{}]".format(contact.collision1, contact.collision2))

            for (position, normal, depth) in zip(contact.position, contact.normal, contact.depth):
                print("Position: {} {} {}".format(position.x, position.y, position.z))
                print("Normal: {} {} {}".format(normal.x, normal.y, normal.z))
                print("Depth: {}".format(depth))
