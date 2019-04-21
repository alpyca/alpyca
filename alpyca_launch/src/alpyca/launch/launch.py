#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import os
import subprocess
import xml.etree.ElementTree as et

from alpyca.launch.node import Node

__all__ = ['Launch']


class Launch(object):

    def __init__(self):
        self.nodes = []
        self.launches = []

    @classmethod
    def from_file(cls, package_name, launch_filename):
        package_path = subprocess.check_output(['rospack', 'find', package_name]).replace('\n', '')
        launch_path = os.path.join(package_path, 'launch', launch_filename)

        
        tree = et.parse(launch_path)
        root = tree.getroot()

        launch = cls()

        for element in root.findall('node'):
            launch.nodes.append(Node.from_xml_element(element))

        return launch

    def add_node(self, node):
        self.nodes.append(node)

    def add_launch(self, sub_launch):
        self.launches.append(sub_launch)

    def append_nodes(self, nodes):
        for node in self.nodes:
            nodes[node.node_name] = node
        for launch in self.launches:
            launch.append_nodes(nodes)
