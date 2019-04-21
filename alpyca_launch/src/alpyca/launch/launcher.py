#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import itertools
import time
import types

import rospy
from alpyca_launch.msg import State, NodeState
from alpyca_launch.srv import StartStop, StartStopRequest, StartStopResponse

__all__ = ['Launcher']


class Launcher(object):

    def __init__(self):
        self.nodes = []
        self.launchers = []

    def add_node(self, node):
        self.nodes.append(node)

    def add_launcher(self, sub_launcher):
        self.launchers.append(sub_launcher)

    def _run(self):
        for node in self.nodes:
            node.start()
        for launcher in self.launchers:
            launcher._run()

    def iter_all_nodes(self):
        all_nodes = [self.nodes, ]
        for launcher in self.launchers:
            all_nodes.append(launcher.iter_all_nodes())

        return itertools.chain.from_iterable(all_nodes)

    def start_node(self, node_name):
        for node in self.iter_all_nodes():
            if node.node_name == node_name:
                node.start()

    def stop_node(self, node_name):
        for node in self.iter_all_nodes():
            if node.node_name == node_name:
                node.stop()

    def start_stop(self, req):
        if req.action == StartStopRequest.START:
            self.start_node(req.node)
        elif req.action == StartStopRequest.STOP:
            self.stop_node(req.node)
        else:
            pass
            # TODO: Log bad action

        return StartStopResponse()

    def run(self):
        self._run()

        rospy.init_node('alpyca_launch')
        pub = rospy.Publisher('launch_topic', State, queue_size=5)
        start_stop_service = rospy.Service('start_stop', StartStop, self.start_stop)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            nodes = []
            for node in self.iter_all_nodes():
                node_state = NodeState()
                node_state.name = node.node_name
                nodes.append(node_state)
            state = State()
            state.nodes = nodes

            pub.publish(state)
            rate.sleep()
