#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

__all__ = ['Launch']


class Launch(object):

    def __init__(self):
        self.nodes = []
        self.launches = []

    def add_node(self, node):
        self.nodes.append(node)

    def add_launch(self, sub_launch):
        self.launches.append(sub_launch)

    def append_nodes(self, nodes):
        for node in self.nodes:
            nodes[node.node_name] = node
        for launch in self.launches:
            launch.append_nodes(nodes)
