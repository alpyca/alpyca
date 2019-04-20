#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import time

import rospy
import std_msgs.msg

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
            node.run()
        for launcher in self.launchers:
            launcher._run()

    def run(self):
        self._run()

        rospy.init_node('alpyca_launch')
        pub = rospy.Publisher('launch_topic', std_msgs.msg.String, queue_size=5)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pub.publish(std_msgs.msg.String('hello world'))
            rate.sleep()
