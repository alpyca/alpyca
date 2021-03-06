#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import shlex
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
from alpyca.launch.parsing_exception import ParsingException


def env(name):
    return os.environ[name]


def simple_find(package):
    try:
        return subprocess.check_output(['rospack', 'find', package]).replace('\n', '')
    except subprocess.CalledProcessError:
        raise ParsingException('Unknown package ' + package)

def find_in_directory(directory, name):
    for (dirpath, _, filenames) in os.walk(directory):
        path = dirpath + os.sep + name
        if os.path.isfile(path):
            dir_path = path[:-len(name)-1]
            return dir_path

def find(package, name):
    # Use simple_find if there is another substitution argument in the filename.
    if '$(' in name:
        return simple_find(package)

    env_cmake = os.environ['CMAKE_PREFIX_PATH'].split(':')
    catkin_workspaces = [path for path in env_cmake if os.path.isfile(path + os.sep + '.catkin')]

    for workspace in catkin_workspaces:
        for search_folder in ['lib', 'share']:
            package_path = workspace + os.sep + search_folder + os.sep + package
            path = find_in_directory(package_path, name)
            if path is not None:
                return path

    package_dir = simple_find(package)

    file_path = package_dir + os.sep + name
    if os.path.isfile(file_path):
        return package_dir

    path = find_in_directory(package_dir, name)
    if path is not None:
        return path

    raise ParsingException('package {} with file {} not found'.format(package, name))

def optenv(value):
    words = value.split(' ')
    env_name = words[0]
    if env_name in os.environ:
        return os.environ[env_name]
    else:
        return ' '.join(words[1:])


def anon(value):
    return 'anon' + str(hash(value))


Match = namedtuple('Match', ['start', 'end', 'keyword', 'value', 'replacement'])
Remap = namedtuple('Remap', ['from_topic', 'to_topic'])


class Degrees(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = '!degrees'
    def __init__(self, val):
        self.val = val

    @classmethod
    def from_yaml(cls, loader, node):
        return eval(node.value + '*pi/180.0', None, {'pi': math.pi})


class Radians(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = '!radians'
    def __init__(self, val):
        self.val = val

    @classmethod
    def from_yaml(cls, loader, node):
        return eval(node.value, None, {'pi': math.pi})


class Substitutor(object):

    def __init__(self, dirname, args=None):
        self.dirname = dirname

        if args is None:
            self.args = {}
        else:
            self.args = args

        self.actions = {'arg': self.arg,
                        'dirname': self.dirname_func,
                        'env': env,
                        'optenv': optenv,
                        'anon': anon}

    def eval_text(self, text):
        locals = dict(self.args)
        locals.update(self.actions)
        locals['find'] = simple_find
        locals['pi'] = math.pi
        locals['true'] = True
        locals['false'] = False
        return str(eval(text, None, locals))

    def arg(self, name):
        value = name.replace(' ', '')
        if value not in self.args:
            raise ParsingException('Unknown argument {}!'.format(value))
        return self.args[value]

    def dirname_func(self, _):
        return self.dirname

    def find_eval_action(self, text):
        expr = r'\$\(eval (.*?)\)'

        matches = []
        for re_match in re.finditer(expr, text):
            start = re_match.start()
            end = re_match.end()

            words = text[start+2:end-1].split(' ')
            keyword = words[0]
            value = ' '.join(words[1:])

            # As a limitation, $(eval) expressions need to span the whole attribute string.
            # A mixture of other substitution args with eval within a single string is not possible.
            if text.startswith('$(eval') and text.endswith(')'):
                end = len(text)
                words = text[start+2:end-1].split(' ')
                keyword = words[0]
                value = ' '.join(words[1:])
                replacement = self.eval_text(value)
            else:
                raise ParsingException('$(eval) expressions need to span the whole attribute string!')

            match = Match(start, end, keyword, value, replacement)
            matches.append(match)
        
        return matches

    def find_find_action(self, text):
        expr = r'\$\(find (.*?)\)'

        matches = []
        for re_match in re.finditer(expr, text):
            start = re_match.start()
            end = re_match.end()

            words = text[start+2:end-1].split(' ')
            keyword = words[0]
            value = ' '.join(words[1:])

            if len(text) > end and text[end] == '/':
                end_postfix = end + 1
                while not (len(text) == end_postfix or text[end_postfix] in [' ', '\'', '\"']):
                    end_postfix += 1
                replacement = find(value, text[end+1:end_postfix])
            else:
                replacement = simple_find(value)
            match = Match(start, end, keyword, value, replacement)
            matches.append(match)
        
        return matches

    def find_action(self, text, action):
        if action == 'eval':
            return self.find_eval_action(text)
        elif action == 'find':
            return self.find_find_action(text)

        expr = r'\$\(' + action + '(.*?)\)'

        matches = []
        for re_match in re.finditer(expr, text):
            start = re_match.start()
            end = re_match.end()

            words = text[start+2:end-1].split(' ')
            keyword = words[0]
            value = ' '.join(words[1:])

            if keyword in self.actions:
                replacement = self.actions[keyword](value)
            else:
                raise ParsingException('Unknown keyword {}!'.format(keyword))
            match = Match(start, end, keyword, value, replacement)
            matches.append(match)
        
        return matches

    def find_unknown_action(self, text):
        expr = r'\$\((.*?)\)'

        matches = []
        for re_match in re.finditer(expr, text):
            raise ParsingException('TODO')

    def substitute_arg(self, text, force_type=None):
        for action in ['arg', 'dirname', 'env', 'optenv', 'anon', 'find', 'eval']:
            matches = self.find_action(text, action)
            text = Substitutor.replace(text, matches)
        sub_text = text
        self.find_unknown_action(sub_text)

        if force_type is None:
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
        else:
            sub_text = force_type(sub_text)

        return sub_text

    def substitute_element_value(self, element, value, force_type=None):
        arg = element.get(value)
        if arg is None:
            raise ParsingException('Missing value {}'.format(value))
        return self.substitute_arg(element.get(value).encode('unicode_escape'), force_type=force_type)

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

    def __init__(self, root=None, sub=None, ns=None):
        self.nodes = {}
        self.launches = []
        self.params = {}
        self.remaps = {}
        self.envs = {}
        self.ns = ns

        if root is None:
            root = {}
        if sub is None:
            sub = Substitutor('')

        if 'ns' in root.keys():
            group_ns = sub.substitute_element_value(root, 'ns')
            if self.ns == None:
                if group_ns[0] == '/':
                    self.ns = group_ns
                else:
                    self.ns = '/' + group_ns
            else:
                self.ns += '/' + group_ns

        self.args = sub.args

        for element in root:
            if element.tag == 'include':
                pass_all_args = True
                if 'pass_all_args' in element.keys():
                    pass_all_args = element.get('pass_all_args').lower() == 'true'
                if pass_all_args:
                    launch = Launch.from_xml_element(element, sub, dict(self.args))
                else:
                    launch = Launch.from_xml_element(element, sub)
                self.launches.append(launch)
                self.nodes.update(launch.nodes)
                self.params.update(launch.params)
            elif element.tag == 'arg':
                name = element.get('name')
                if 'default' in element.keys():
                    if name in self.args:
                        value = self.args[name]
                    else:
                        value = sub.substitute_element_value(element, 'default')
                elif 'value' in element.keys():
                    value = sub.substitute_element_value(element, 'value')
                else:
                    # value has to be given by constructor, or other launch file
                    continue
                self.args[name] = value
            elif element.tag == 'env':
                name = element.get('name')
                value = sub.substitute_element_value(element, 'value')
                self.envs[name] = value
            elif element.tag == 'group':
                launch = Launch(element, sub, self.ns)
                self.launches.append(launch)

                self.nodes.update(launch.nodes)
                self.params.update(launch.params)
            elif element.tag == 'node':
                remaps = dict(self.remaps)

                package_name = sub.substitute_element_value(element, 'pkg')
                executable = sub.substitute_element_value(element, 'type')
                node_name = sub.substitute_element_value(element, 'name')

                extra_args = None
                respawn = False
                respawn_delay = 0.0
                required = False
                ns = None
                clear_params = False
                working_directory = None
                launch_prefix = None

                if 'args' in element.keys():
                    extra_args = shlex.split(sub.substitute_element_value(element, 'args'))
                if 'respawn' in element.keys():
                    respawn = sub.substitute_element_value(element, 'respawn')
                if 'respawn_delay' in element.keys():
                    respawn_delay = sub.substitute_element_value(element, 'respawn_delay')
                if 'required' in element.keys():
                    required = sub.substitute_element_value(element, 'required')
                if 'ns' in element.keys():
                    ns = sub.substitute_element_value(element, 'ns')
                if 'clear_params' in element.keys():
                    clear_params = sub.substitute_element_value(element, 'clear_params')
                if 'cwd' in element.keys():
                    working_directory = sub.substitute_element_value(element, 'cwd')
                if 'launch-prefix' in element.keys():
                    launch_prefix = sub.substitute_element_value(element, 'launch-prefix').split(' ')

                if ns is None and self.ns is None:
                    pass
                elif ns is not None and self.ns is None:
                    pass
                elif ns is None and self.ns is not None:
                    ns = self.ns
                else:
                    ns = self.ns + '/' + ns

                if ns is not None and ns[0] != '/':
                    ns = '/' + ns

                for sub_element in element:
                    if sub_element.tag == 'remap':
                        remaps[sub.substitute_element_value(sub_element, 'from')] = sub.substitute_element_value(sub_element, 'to')
                    elif sub_element.tag == 'param':
                        param = Param.from_xml_element(sub_element, sub, ns, node_name)
                        if isinstance(param, Param):
                            self.params[param.name] = param
                        elif isinstance(param, dict):
                            self.params.update(param)
                node = Node(package_name, executable, node_name, ns=ns, remaps=remaps, envs=dict(self.envs), extra_args=extra_args, respawn=respawn, respawn_delay=respawn_delay, required=required, clear_params=clear_params, working_directory=working_directory, launch_prefix=launch_prefix)
                self.add_node(node)
            elif element.tag == 'test':
                raise ParsingException('test tag is not supported!')
            elif element.tag == 'machine':
                raise ParsingException('machine tag is not supported!')
            elif element.tag == 'param':
                param = Param.from_xml_element(element, sub, self.ns)
                if isinstance(param, Param):
                    self.params[param.name] = param
                elif isinstance(param, dict):
                    self.params.update(param)
            elif element.tag == 'remap':
                from_topic = sub.substitute_element_value(element, 'from')
                to_topic = sub.substitute_element_value(element, 'to')

                self.remaps[from_topic] = to_topic
            elif element.tag == 'rosparam':
                subst_value = True
                if 'subst_value' in element.keys():
                    subst_value = element.get('subst_value').lower() == 'true'

                if 'file' in element.keys():
                    with open(sub.substitute_element_value(element, 'file'), 'r') as yaml_file:
                        text = yaml_file.read()
                else:
                    text = element.text

                if subst_value:
                    text = sub.substitute_arg(text)
                
                name = ''
                if 'param' in element.keys():
                    name = element.get('param')
                if 'ns' in element.keys():
                    ns = element.get('ns')
                    if name == '':
                        name = ns
                    else:
                        name = ns + '/' + name

                if len(name) > 0 and name[0] != '/':
                    name = '/' + name

                if text.replace('\n', '').replace('\t', '').replace(' ', '') != '':
                    parameters = Param.from_dicts(yaml.safe_load(text), name)
                    self.params.update(parameters)
            else:
                raise ParsingException('Unknown tag {}!'.format(element.tag))

    @classmethod
    def from_launch(cls, package_name, launch_filename, args=None):
        launch_path = cls.find_launch(package_name, launch_filename)
        return cls.from_launch_filename(launch_path, args)

    @classmethod
    def from_xml_element(cls, element, sub, args=None):
        filename = sub.substitute_element_value(element, 'file')
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
        try:
            root = et.fromstring(string)
        except et.ParseError as err:
            raise ParsingException(err.msg)
        launch = cls(root, sub)
        return launch

    @staticmethod
    def find_launch(package_name, launch_filename):
        package_path = simple_find(package_name)
        launch_path = os.path.join(package_path, 'launch', launch_filename)
        if not os.path.isfile(launch_path):
             launch_path = os.path.join(package_path, launch_filename)
        return launch_path

    def add_node(self, node):
        full_name = node.full_name
        if full_name in self.nodes:
            raise ParsingException('Node name is not unique: {}'.format(full_name))
        self.nodes[full_name] = node

    def add_launch(self, sub_launch):
        self.launches.append(sub_launch)

    def append_nodes(self, nodes):
        nodes.update(self.nodes)
        for launch in self.launches:
            launch.append_nodes(nodes)

    def get_all_params(self):
        all_params = dict(self.params)
        for launch in self.launches:
            all_params.update(launch.get_all_params())
        return all_params
