
#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import subprocess
import yaml
import re
import math

from alpyca.launch.parsing_exception import ParsingException


def bool_converter(value):
    if value.lower() == 'true':
        return True
    elif value.lower() == 'false':
        return False
    else:
        raise ValueError('value {} is not a boolean!'.format(value))


class Param(object):

    def __init__(self, name, value):
        self.name = name
        self.value = value

        self.subst_deg()
        self.subst_rad()

    def subst_deg(self):
        if isinstance(self.value, str):
            expr = r'deg\((.*?)\)'
            match = re.search(expr, self.value)
            if match is not None:
                self.value = eval(match.group(1), None, {'pi': math.pi}) * math.pi / 180.0

    def subst_rad(self):
        if isinstance(self.value, str):
            expr = r'rad\((.*?)\)'
            match = re.search(expr, self.value)
            if match is not None:
                self.value = eval(match.group(1), None, {'pi': math.pi})

    @staticmethod
    def from_dicts(collection, ns=''):
        parameters = {}
        if isinstance(collection, dict):
            for param in [Param.from_dicts(value, ns + '/' + key) for key, value in collection.items()]:
                parameters.update(param)
        elif ns != '':
            parameters[ns] = Param(ns, collection)

        return parameters

    @classmethod
    def from_xml_element(cls, element, sub, ns=None, node_name=None):
        condition = True
        if 'if' in element.keys() and 'unless' in element.keys():
            raise ParsingException('Param tag contains if and unless at the same time!')
        elif 'if' in element.keys():
            if_elem = sub.substitute_element_value(element, 'if')
            if isinstance(if_elem, (bool, int)):
                condition = bool(if_elem)
            else:
                raise ParsingException('Unknown value')
        elif 'unless' in element.keys():
            unless_elem = sub.substitute_element_value(element, 'unless')
            if isinstance(unless_elem, (bool, int)):
                condition = not bool(unless_elem)
            else:
                raise ParsingException('Unknown value')

        if condition:
            if 'name' not in element.keys():
                raise ParsingException('Missing tag name in param!')
            name = element.get('name')

            if name[0] == '~':
                name = name[1:]

            if name[0] != '/':
                if ns is None and node_name is None:
                    name = '/' + name
                elif ns is not None and node_name is None:
                    name = ns + '/' + name
                elif ns is None and node_name is not None:
                    name = '/' + node_name + '/' + name
                else:
                    name = ns + '/' + node_name + '/' + name
            elif node_name is not None:
                name = '/' + node_name + name

            name = name.replace('//', '/')

            force_type = None
            if 'type' in element.keys():
                
                force_type = sub.substitute_element_value(element, 'type')
                if force_type == 'int':
                    force_type = int
                elif force_type == 'double':
                    force_type = float
                elif force_type == 'str':
                    force_type = str
                elif force_type == 'boolean' or force_type == 'bool':
                    force_type = bool_converter
                elif force_type == 'yaml':
                    if 'value' in element.keys():
                        yaml_value = sub.substitute_element_value(element, 'value')
                    else: # 'command' in element.keys()
                        cmd = element.get('command').split(' ')
                        try:
                            yaml_value = subprocess.check_output(cmd)
                        except subprocess.CalledProcessError:
                            raise ParsingException('Unknown command {}!'.format(cmd))

                    if isinstance(yaml_value, (int, bool)):
                        return cls(name, yaml_value)
                    else:
                        yaml_params = yaml.load(yaml_value)
                        if isinstance(yaml_params, dict):
                            params = {}
                            for key, value in yaml_params.items():
                                param_name = name + '/' + key
                                param = cls(param_name, value)
                                params[param_name] = param
                            return params
                        else:
                            raise ParsingException('Unknown yaml command {}!'.format(yaml_params))
                    return
                else:
                    raise ParsingException('Unknown type {}!'.format(force_type))

            def exists(name):
                tag_exists = name in element.keys()
                if not tag_exists:
                    return False

                for cmd in ['value', 'textfile', 'binfile', 'command']:
                    if name != cmd and cmd in element.keys():
                        raise ParsingException('Invalid tags {} and {}!'.format(name, cmd))

                return True

            if exists('value'):
                value = sub.substitute_element_value(element, 'value', force_type=force_type)
            elif exists('textfile'):
                path = sub.substitute_element_value(element, 'textfile', force_type=str)
                with open(path, 'r') as file:
                    value = file.read()
            elif exists('binfile'):
                path = sub.substitute_element_value(element, 'binfile', force_type=str)
                with open(path, 'rb') as file:
                    value = file.read()
            elif exists('command'):
                cmd = element.get('command').split(' ')
                try:
                    value = subprocess.check_output(cmd)
                except subprocess.CalledProcessError:
                    raise ParsingException('Unknown command {}!'.format(cmd))
            else:
                raise ParsingException('The tags value, textfile or command are missin in param!')
            return cls(name, value)

    def start(self):
        subprocess.check_output(['rosparam', 'set', self.name, str(self.value)])
