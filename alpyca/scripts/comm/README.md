# ros_pythonic

A pythonic wrapper to simplify the communication in ROS.


The process of setting up the communication in ROS can be hard for a beginner. To simplify this process and also provide a more pythonic interface for ROS this project provides a flask like approach to set up the communication in ROS.

## Example usage
Lets subscribe and publish something as an example.

```python
from ros_pythonic import Node

# create a node instance
node = Node()

# use the topic_sub decorator to subscribe to the topic
# /test_topic/sub that uses the data type 'std_msgs.msg.String'
@node.topic_sub('/test_topic/sub', 'std_msgs.msg.String')
def sub_callback(msg):
    print('callback from sub_callback: {0}'.format(msg))

# use the topic_pub decorator to publish data with the data type
# the data type 'std_msgs.msg.String' to the topic /test_topic/pub 
@node.topic_pub('/test_topic/pub', 'std_msgs.msg.String')
def pub_given_data(data):
    return data

# the data to publish doesn't have to be given as an argument but can
# also be created in the function itself
@node.topic_pub('/test_topic/pub', 'std_msgs.msg.String')
def pub_data_intern():
    return 'intern generated random data'

# optional main method of the the node that gets called periodically
@node.main()
def main():
    pub_given_data('given test data')
    pub_data_intern()


# starts the node and the communication
if __name__ == "__main__":
    node.run()
```
