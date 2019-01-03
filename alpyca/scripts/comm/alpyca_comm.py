from __future__ import division, absolute_import, print_function

import uuid
import rospy
import functools

__all__ = ['Node']


class Node(object):
    """ Convenience class to simplify the communication within ROS """

    def __init__(self):
        self.topic_subs = []

    @staticmethod
    def _import_from_str(import_str):
        """ Import the given string.
        
        Parameters
        ----------
        import_str : str
            string in an importable form, e.g. 'std_msgs.msg.String'
        
        Returns
        -------
        class
            imported class from the given string
        """
        try:
            from_part, imp_part = import_str.rsplit('.', 1)
            mod = __import__(from_part, fromlist=[imp_part])
            return getattr(mod, imp_part)
        except ImportError:
            rospy.logerr('Cant import dtype: {}'.format(import_str))
            raise

    def topic_sub(self, sub_topic, msg_type):
        """ Adds the given topic to the list of subscriptions.
        
        Parameters
        ----------
        sub_topic : str
            topic to subscribe to, e.g. /test_topic/sub
        msg_type : str
            data type of the topic in an importable form, 
            e.g. 'std_msgs.msg.String'
        
        Returns
        -------
        function
            callback of the function to call when message 
            arrives on suscribed topic
        """
        def wrapper(callback):
            imported_dtype = self._import_from_str(msg_type)
            self.topic_subs.append((sub_topic,
                                    imported_dtype,
                                    callback))
        return wrapper

    def topic_pub(self, pub_topic, msg_type, queue_size=10):
        """ Enables publishing to the given topic if the decorated 
        function is called.
        
        Parameters
        ----------
        pub_topic : str
            topic to publish to, e.g. /test_topic/pub
        msg_type : str
            data type of the topic in an importable form, 
            e.g. 'std_msgs.msg.String'
        queue_size : int, optional
            size of the publisher queue (the default is 10)
        
        Returns
        -------
        function
            callback of the function to call that is called
            to publish the data to the topic
        """
        def cb_decorator(callback):
            # two decorators to enable arg/no arg for pub
            imported_dtype = self._import_from_str(msg_type)
            @functools.wraps(callback)
            def wrapper(*args, **kwargs):
                cb_ret = callback(*args, **kwargs)
                pub = rospy.Publisher(pub_topic,
                                      imported_dtype,
                                      queue_size=queue_size)
                pub.publish(cb_ret)
            return wrapper    
        return cb_decorator    

    def main(self, rate=10):
        """ Optional main function to start to execute the
        decorated function perodically with the given rate.
        
        Parameters
        ----------
        rate : int, optional
            rate to execute perodically (the default is 10)
        
        Returns
        -------
        function
            callback of the function to execute perodically
        """
        def wrapper(callback):
            self.main_runner = callback, rate
        return wrapper

    def run(self, node_name=None):
        """ Spins up the ROS node and starts the communication.
        
        Parameters
        ----------
        node_name : str, optional
            name of the node to start 
            (the default is None; a random name is generated)
        
        """
        if not node_name:  
            node_name_init = 'node_' +  uuid.uuid4().hex.upper()[0:6]
        else:
            node_name_init = node_name

        rospy.init_node(node_name_init, anonymous=True)
        rospy.loginfo('created node: {}'.format(node_name_init))

        # subscriptions have to be done here and not in the callback
        # b/c otherwise ROS doesn't register the subscription w/o
        # a additional call of the function (which executes the function)
        for topic, dtype, cb in self.topic_subs:
            rospy.Subscriber(topic, dtype, cb)
            rospy.loginfo('subscribed to: {}'.format(topic))

        try:
            # only exec main perodically if there is something to do
            ros_rate = rospy.Rate(self.main_runner[1])
            while not rospy.is_shutdown():
                self.main_runner[0]()
                ros_rate.sleep()
        except AttributeError:    
            if self.topic_subs:
                # node only listens on topics - nothing else to do
                rospy.spin()

