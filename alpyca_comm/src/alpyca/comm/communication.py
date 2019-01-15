from __future__ import division, absolute_import, print_function

import sys
import uuid
import rospy
import inspect
import functools

import actionlib

__all__ = ['Node']


class Node(object):
    """ Convenience class to simplify the communication within ROS """

    def __init__(self):
        self.topic_subs = []
        self.action_refs = {}
        self.action_results = {}
        self.action_mapper = {}

    @staticmethod
    def _import_from_str(import_str, asterisk=False):
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

    def service_call(self, srv_name, srv_type):
        """ Calls the given service that has the given service type.
        
        Parameters
        ----------
        srv_name : str
            service to call, e.g. /test_srv/service
        srv_type : str
            data type of the service in an importable form, 
            e.g. 'my_service.srv.ServiceType'
        
        Returns
        -------
        srv_type response
            return the response of the service that was called
        """
        def cb_decorator(callback):
            @functools.wraps(callback)
            def wrapper(*args, **kwargs):
                rospy.wait_for_service(srv_name)
                try:
                    data = args[0]
                    imported_dtype = self._import_from_str(srv_type)
                    srv_call = rospy.ServiceProxy(srv_name, imported_dtype)
                    srv_response = srv_call(data)
                    return callback(srv_response)
                except rospy.ServiceException as srv_err:
                    rospy.logerr("Service call failed: {}".format(srv_err))
                except IndexError:
                    rospy.logerr('Service call argument needs to be type: {}'\
                    .format(srv_type))
            return wrapper    
        return cb_decorator 

    def service_provide(self, srv_name, srv_type):
        """ Provides a topic with the given name and service type.
        
        Parameters
        ----------
        srv_name : str
            service to provide, e.g. /test_srv/service
        srv_type : str
            data type of the service in an importable form, 
            e.g. 'my_service.srv.ServiceType'
        
        Returns
        -------
        function
            callback of the function to call when the provided
            service gets called
        """
        def wrapper(callback):
            imported_dtype = self._import_from_str(srv_type)
            rospy.Service(srv_name, imported_dtype, callback)
        return wrapper

    @staticmethod
    def _get_action_msg_import(act_import, act_type):
        """ Converts the basic action dtype to an importable format.

        After the action gets generated of catkin the import changes, e.g.
        alpyca.action.Example -> alpyca.msg.ExampleAction
        
        Parameters
        ----------
        act_import : str
            Datatype of the action, e.g. 'alpyca.action.Example'
        act_type : str
            Type of the action to make importable, e.g. Action, Feedback, Goal..
        
        Returns
        -------
        str
            Importable action msg, alpyca.msg.ExampleAction
        """
        base_pkg, action_name = act_import.rsplit('.', 1)
        base_pkg_path, _ = base_pkg.rsplit('.', 1)
        return '.'.join([base_pkg_path, 'msg', action_name + act_type])

    @staticmethod
    def _get_attribute_name(class_of_attr):
        """ Gets the attribute name of the given class.

        This is needed to get the attribute names for Goal, Feedback
        and Result of the action.
        
        Parameters
        ----------
        class_of_attr : cls
            Class to get the attribute from.
        
        Returns
        -------
        str
            Name of the searched attribute.
        """
        attributes_of_class = [i for i in dir(class_of_attr) if not inspect.ismethod(i)]
        attr_wo_serials = [i for i in attributes_of_class if 'serial' not in i]
        return [i for i in attr_wo_serials if '_' not in i][0]

    @staticmethod
    def _set_attr_of_action(attr, attr_class, val):
        """ Sets the attribute of a action with the given value.

        This is needed b/c the attributes of the action (Goal, Result, ..) can
        have different names and types which have to be determined an set differntly.
        
        Parameters
        ----------
        attr : str
            Name of the class attribute to set.
        attr_class : cls
            Class the attribute belongs to
        val : dtype of value to set
            Value to set the attribute to.
        """
        attr_ref = getattr(attr_class, attr)
        if isinstance(attr_ref, int) or isinstance(attr_ref, str):
            attr_ref = val
        else:
            attr_ref.append(val)

    @staticmethod
    def _get_action_dtypes(act_type):
        """ Determines the three types (Goal, Feedback, Result) of an 
        action in a callable format.
        
        Parameters
        ----------
        act_type : act_type : str
            Datatype of the action, e.g. 'alpyca.action.Example'
        
        Returns
        -------
        (type, type, type)
            Callable type of Goal, Feedback and Result of the action
        """
        action_msg_import = Node._get_action_msg_import(act_type, 'Action')
        action_dtype = Node._import_from_str(action_msg_import)

        result_msg_import = Node._get_action_msg_import(act_type, 'Result')
        result_dtype = Node._import_from_str(result_msg_import)

        feedback_msg_import = Node._get_action_msg_import(act_type, 'Feedback')
        feedback_dtype = Node._import_from_str(feedback_msg_import)
        return action_dtype, result_dtype, feedback_dtype


    def start_action_server(self, act_name, act_type):
        """ Sets everything up to enable starting of the action server.
        
        Parameters
        ----------
        act_name : str
            Topic to access the action, e.g. '/test_act/name'
        act_type : str
            Datatype of the action, e.g. 'alpyca.action.Example'
        """
        def wrapper(callback):
            # get data types of action, result and feedback
            act_dtype, res_dtype, feedk_dtype = self._get_action_dtypes(act_type)
            # create action server
            act_srv = actionlib.SimpleActionServer(act_name,
                                                   act_dtype, 
                                                   execute_cb=callback, 
                                                   auto_start = False)
            # create result and feedback references
            result_ref = res_dtype()
            feedb_ref = feedk_dtype()
            
            # map action client <-> action server
            self.action_mapper[callback.__name__] = act_name
            # save references for further access
            self.action_refs[act_name] = (act_srv, result_ref, feedb_ref)
        return wrapper

    def set_action_result(self, result):
        """ Sets the result for the action server that gets decorated.
        
        Parameters
        ----------
        result : dtype of the action
            Result to set for the action
        """
        act_method_name = sys._getframe().f_back.f_code.co_name
        act_topic = self.action_mapper[act_method_name]
        act_srv, result_ref, _ = self.action_refs[act_topic]
        result_attr = self._get_attribute_name(result_ref)
        self._set_attr_of_action(result_attr, result_ref, result)
        self.action_results[act_topic] = result
        act_srv.set_succeeded(result_ref)

    def send_action_feedback(self, feedback):
        """ Sends a feed for the action server that gets decorated.
        
        Parameters
        ----------
        feedback : dtype of the feedback
            Feedback to send
        """
        act_method_name = sys._getframe().f_back.f_code.co_name
        act_topic = self.action_mapper[act_method_name]
        act_srv, _, feedb_ref = self.action_refs[act_topic]
        feedb_attr = self._get_attribute_name(feedb_ref)
        self._set_attr_of_action(feedb_attr, feedb_ref, feedback)
        act_srv.publish_feedback(feedb_ref)

    def get_action_result(self):
        """ Gets the action result for the decorated action client.
        
        Returns
        -------
        dtype of the result
            Result of the action that gets called from 
            the decorated action client.
        """
        act_method_name = sys._getframe().f_back.f_code.co_name
        return self.action_results[self.action_mapper[act_method_name]]

    def action_client(self, act_name, act_type, feedback_cback=None):
        """ Sets everythin up a action client.
        
        Parameters
        ----------
        act_name : str
            Topic to access the action, e.g. '/test_act/name'
        act_type : str
            Datatype of the action, e.g. 'alpyca.action.Example'
        feedback_cback : function, optional
            Callback of the method that gets called in case of a feedback 
            from the action server (the default is None, which means no feedback
            method gets called).
        """
        def cb_decorator(callback):
            self.action_mapper[callback.__name__] = act_name
            @functools.wraps(callback)
            def wrapper(*args, **kwargs):
                goal_to_reach = args[0]
                action_msg_import = self._get_action_msg_import(act_type, 'Action')
                action_dtype = self._import_from_str(action_msg_import)
                
                # Creates the action client .
                client = actionlib.SimpleActionClient(act_name, action_dtype)

                # Waits until the action server has started up and started
                # listening for goals.
                client.wait_for_server()

                # Creates a goal to send to the action server.
                goal_msg_import = self._get_action_msg_import(act_type, 'Goal')
                goal_dtype = self._import_from_str(goal_msg_import)
                goal = goal_dtype(order=goal_to_reach)

                # Sends the goal to the action server.
                client.send_goal(goal, feedback_cb=feedback_cback)

                # Waits for the server to finish performing the action.
                client.wait_for_result()

                # Executes the decorated method and returns the result
                return callback(*args, **kwargs)
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

        for key in self.action_refs.keys():
            self.action_refs[key][0].start()

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

