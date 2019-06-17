#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import os
import re
import subprocess
import xml.etree.ElementTree as et
from collections import namedtuple
from functools import partial

from alpyca.launch.node import Node

__all__ = ['Launch']


def find_package(package_name):
    return subprocess.check_output(['rospack', 'find', package_name]).replace('\n', '')


Match = namedtuple('Match', ['start', 'end', 'keyword', 'value', 'replacement'])


class Substitutor(object):

    def __init__(self, args):
        self.args = args



    def find_action(self, text):
        expr = r'\$\((.*?)\)'

        matches = []
        for re_match in re.finditer(expr, text):
            start = re_match.start()
            end = re_match.end()
            keyword, value = text[start+2:end-1].split(' ')

            if keyword == 'find':
               replacement = find_package(value)
            elif keyword == 'arg':
               replacement = self.args[value]
            else:
               # TODO: Throw exception
               pass
            match = Match(start, end, keyword, value, replacement)
            matches.append(match)
        
        return matches

    def substitute_arg(self, text):
        print('before: ' + text)
        matches = self.find_action(text)
        sub_text = Substitutor.replace(text, matches)
        print('after: ' + sub_text)
        return sub_text

    @staticmethod
    def replace(text, matches):
        if len(matches) == 0:
            return text

        text_list = []
        for index in range(len(matches)):
            if index == 0:
                text_list.append(text[:matches[0].start])
            else:
                text_list.append(text[matches[index-1].end:matches[index].start])
            text_list.append(matches[index].replacement)

        text_list.append(text[matches[-1].end:])
        text = ''.join(text_list)

        return text



class Launch(object):

    def __init__(self):
        self.nodes = []
        self.launches = []
        self.args = {}

    @classmethod
    def from_launch(cls, package_name, launch_filename):
        launch_path = cls.find_launch(package_name, launch_filename)
        return cls.from_launch_filename(launch_path)

    @classmethod
    def from_xml_element(cls, element, sub):
        filename = sub.substitute_arg(element.get('file'))
        return cls.from_launch_filename(filename)

    @classmethod
    def from_launch_filename(cls, filename):
        with open(filename, 'r') as file:
            text = file.read()

        launch = cls()
        sub = Substitutor(launch.args)

        tree = et.parse(filename)
        root = tree.getroot()

        # TODO: include, arg, env, group, include, node, test, machine, param, remap, rosparam
        for element in root:
            if element.tag == 'include':
                launch.launches.append(Launch.from_xml_element(element, sub))
            elif element.tag == 'arg':
                name = element.get('name')
                default = element.get('default')
                value = sub.substitute_arg(default)
                launch.args[name] = value
            elif element.tag == 'env':
                pass
            elif element.tag == 'group':
                pass
            elif element.tag == 'node':
                package_name = sub.substitute_arg(element.get('pkg'))
                executable = sub.substitute_arg(element.get('type'))
                node_name = sub.substitute_arg(element.get('name'))
                launch.nodes.append(Node(package_name, executable, node_name))
            elif element.tag == 'test':
                pass
            elif element.tag == 'machine':
                pass
            elif element.tag == 'param':
                pass
            elif element.tag == 'remap':
                pass
            elif element.tag == 'rosparam':
                pass
            else:
                pass
                # TODO: Throw exception

        return launch

    @staticmethod
    def find_launch(package_name, launch_filename):
        package_path = find_package(package_name)
        launch_path = os.path.join(package_path, 'launch', launch_filename)
        return launch_path

    def add_node(self, node):
        self.nodes.append(node)

    def add_launch(self, sub_launch):
        self.launches.append(sub_launch)

    def append_nodes(self, nodes):
        for node in self.nodes:
            nodes[node.node_name] = node
        for launch in self.launches:
            launch.append_nodes(nodes)
