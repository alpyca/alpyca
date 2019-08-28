#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import yaml
import math
import os
import re
import subprocess
import xml.etree.ElementTree as et
from collections import namedtuple
from functools import partial
from itertools import chain

from alpyca.launch.node import Node
from alpyca.launch.param import Param

__all__ = ['Launch', 'ParsingExeption']


def find_package(package_name):
    return subprocess.check_output(['rospack', 'find', package_name]).replace('\n', '')


Match = namedtuple('Match', ['start', 'end', 'keyword', 'value', 'replacement'])
Remap = namedtuple('Remap', ['from_topic', 'to_topic'])


class ParsingException(Exception):
    pass


class Substitutor(object):

    def __init__(self, dirname, args=None):
        self.dirname = dirname

        if args is None:
            self.args = {}
        else:
            self.args = args

    def find_action(self, text):
        expr = r'\$\((.*?)\)'

        matches = []
        for re_match in re.finditer(expr, text):
            start = re_match.start()
            end = re_match.end()

            words = text[start+2:end-1].split(' ')
            keyword = words[0]
            value = ' '.join(words[1:])

            if keyword == 'find':
                replacement = find_package(value)
            elif keyword == 'arg':
                value = value.replace(' ', '')
                if value not in self.args:
                    raise ParsingException('Unknown argument {}!'.format(value))
                replacement = self.args[value]
            elif keyword == 'eval':
                eval_args = dict(self.args)
                eval_args['pi'] = math.pi
                replacement = str(eval(value, None, eval_args))
            elif keyword == 'dirname':
                replacement = self.dirname
            else:
                # TODO: Throw exception
                pass
            match = Match(start, end, keyword, value, replacement)
            matches.append(match)
        
        return matches

    def substitute_arg(self, text):
        matches = self.find_action(text)
        sub_text = Substitutor.replace(text, matches)
        if sub_text.lower() == 'true':
            sub_text = True
        elif sub_text.lower() == 'false':
            sub_text = False
        else:
            try:
                sub_text = int(sub_text)
            except ValueError:
                try:
                    sub_text = float(sub_text)
                except ValueError:
                    pass
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

        text_list = [str(text) for text in text_list]
        text = ''.join(text_list)

        return text



class Launch(object):

    def __init__(self, root, sub):
        self.nodes = []
        self.launches = []
        self.params = []
        self.remaps = {}

        self.args = sub.args

        # TODO: include, arg, env, group, include, node, test, machine, param, remap, rosparam
        for element in root:
            if element.tag == 'include':
                pass_all_args = False
                if 'pass_all_args' in element.keys():
                    pass_all_args = element.get('pass_all_args').lower() == 'true'
                if pass_all_args:
                    self.launches.append(Launch.from_xml_element(element, sub, dict(self.args)))
                else:
                    self.launches.append(Launch.from_xml_element(element, sub))
            elif element.tag == 'arg':
                name = element.get('name')
                if 'default' in element.keys():
                    if name in self.args:
                        value = self.args[name]
                    else:
                        value = sub.substitute_arg(element.get('default'))
                elif 'value' in element.keys():
                    value = sub.substitute_arg(element.get('value'))
                else:
                    # value has to be given by constructor, or other launch file
                    continue
                self.args[name] = value
            elif element.tag == 'env':
                pass
            elif element.tag == 'group':
                pass
            elif element.tag == 'node':
                remaps = dict(self.remaps)
                params = {}
                for sub_element in element:
                    if sub_element.tag == 'remap':
                        remaps[sub.substitute_arg(sub_element.get('from'))] = sub.substitute_arg(sub_element.get('to'))
                    elif sub_element.tag == 'param':
                        params[sub.substitute_arg(sub_element.get('name'))] = sub.substitute_arg(sub_element.get('value'))
                package_name = sub.substitute_arg(element.get('pkg'))
                executable = sub.substitute_arg(element.get('type'))
                node_name = sub.substitute_arg(element.get('name'))
                self.nodes.append(Node(package_name, executable, node_name, params=params, remaps=remaps))
            elif element.tag == 'test':
                pass
            elif element.tag == 'machine':
                pass
            elif element.tag == 'param':
                name = element.get('name')

                if 'value' in element.keys():
                    value = sub.substitute_arg(element.get('value'))
                else: # 'command' in element.keys()
                    value = subprocess.check_output(sub.substitute_arg(element.get('command')).split(' '))

                param = Param(name, value)
                self.params.append(param)
            elif element.tag == 'remap':
                from_topic = sub.substitute_arg(element.get('from'))
                to_topic = sub.substitute_arg(element.get('to'))

                self.remaps[from_topic] = to_topic
            elif element.tag == 'rosparam':
                subst_value = True
                if 'subst_value' in element.keys():
                    subst_value = element.get('subst_value').lower() == 'true'

                if 'file' in element.keys():
                    with open(sub.substitute_arg(element.get('file')), 'r') as yaml_file:
                        text = yaml_file.read()
                else:
                    text = element.text

                if subst_value:
                    text = sub.substitute_arg(text)
                
                if text.replace('\n', '').replace('\t', '') != '':
                    parameters = Param.from_dicts(yaml.safe_load(text))
                    self.params += parameters
            else:
                pass
                # TODO: Throw exception

    @classmethod
    def from_launch(cls, package_name, launch_filename, args=None):
        launch_path = cls.find_launch(package_name, launch_filename)
        return cls.from_launch_filename(launch_path, args)

    @classmethod
    def from_xml_element(cls, element, sub, args=None):
        filename = sub.substitute_arg(element.get('file'))
        return cls.from_launch_filename(filename, args)

    @classmethod
    def from_launch_filename(cls, filename, args=None):
        sub = Substitutor(os.path.dirname(filename), args)
        tree = et.parse(filename)
        root = tree.getroot()
        launch = cls(root, sub)
        return launch

    @classmethod
    def from_string(cls, string, filename='', args=None):
        sub = Substitutor(os.path.dirname(filename), args)
        root = et.fromstring(string)
        launch = cls(root, sub)
        return launch

    @staticmethod
    def find_launch(package_name, launch_filename):
        package_path = find_package(package_name)
        launch_path = os.path.join(package_path, 'launch', launch_filename)
        if not os.path.isfile(launch_path):
             launch_path = os.path.join(package_path, launch_filename)
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

    def get_all_params(self):
        all_params = [self.params]
        for launch in self.launches:
            all_params.append(launch.get_all_params())

        return list(chain.from_iterable(all_params))
