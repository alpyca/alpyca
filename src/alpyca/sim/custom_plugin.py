#!/usr/bin/env python


class CustomPlugin:

    def __init__(self):
        self.sensor = None

    def Load(self, sensor):
        self.sensor = sensor

        sensor.ConnectUpdated(self.OnUpdate)
        sensor.SetActive(True)

    def OnUpdate(self):

        for contact in self.sensor.Contacts():
            print("Collision between[{}] and [{}]".format(contact.collision1, contact.collision2))

            for (position, normal, depth) in zip(contact.position, contact.normal, contact.depth):
                print("Position: {} {} {}".format(position.x, position.y, position.z))
                print("Normal: {} {} {}".format(normal.x, normal.y, normal.z))
                print("Depth: {}".format(depth))
