# alpyca_sim

**Warning: This project is WIP and does not deliver interfaces for all kinds of plugins yet!**  

Gazebo is a powerful robot simulator. However, there is a steep learning curve to overcome. Even after you know what you are doing, 
writing plugins keeps being a hard task. Error messages are often cryptic, debugging is hard and you have to wait for your code to compile. 
It would be much nicer to write plugins in Python. This is what alpyca/sim is for.  

## Example usage
You can write a contact sensor plugin, exactly as in the [Gazebo tutorials](http://gazebosim.org/tutorials?tut=contact_sensor), but completely in Python. 

```python
class ContactPlugin:

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

```
