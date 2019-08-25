#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import rospy
from alpyca_launch.msg import State, NodeState
from alpyca_launch.srv import StartStop, StartStopRequest, StartStopResponse

__all__ = ['Runner']


class Runner(object):

    def __init__(self):
        pass

    def start_stop(self, req):
        if req.action == StartStopRequest.START:
            self.nodes[req.node].start()
        elif req.action == StartStopRequest.STOP:
            self.nodes[req.node].stop()
        else:
            pass
            # TODO: Log bad action

        return StartStopResponse()

    def run(self, launch):
        self.nodes = {}
        launch.append_nodes(self.nodes)

        for node in self.nodes.values():
            node.start()
        for param in launch.get_all_params():
            param.start()

        rospy.init_node('alpyca_launch')
        pub = rospy.Publisher('launch_topic', State, queue_size=5)
        start_stop_service = rospy.Service('start_stop', StartStop, self.start_stop)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            nodes = []
            for node_name, node in self.nodes.items():
                node_state = NodeState()
                node_state.name = node_name
                nodes.append(node_state)
            state = State()
            state.nodes = nodes

            pub.publish(state)
            rate.sleep()
