#!/usr/bin/env python

import argparse
import os
import re

import CppHeaderParser
from jinja2 import Environment, FileSystemLoader


def convert_snake_case(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def convert_upper_case(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).upper()


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('template_dir', type=str)
    parser.add_argument('binding_path', type=str)
    parser.add_argument('wrapper_path', type=str)
    parser.add_argument('gazebo_include_dirs', type=str,
                        help='Include directories of gazebo. The directories should comma seperated.'
                             'These directories are searched for the header files, for which bindings should be created.')
    args = parser.parse_args()
    return args


def find_gazebo(gazebo_include_dirs):
    for include_dir in gazebo_include_dirs.split(','):
        if os.path.isdir(os.path.join(include_dir, 'gazebo', 'sensors')):
            return os.path.join(include_dir, 'gazebo')

    raise ValueError('Cannot find gazebo header files!')


if __name__ == '__main__':
    args = get_args()
    gazebo_dir = find_gazebo(args.gazebo_include_dirs)
    class Sensor:
        pass
    class Function:
        pass
    class Parameter:
        pass
    sensor = Sensor()
    sensor_path = os.path.join(gazebo_dir, 'sensors', 'ContactSensor.hh')
    cpp_header = CppHeaderParser.CppHeader(sensor_path)
    sensor_class = cpp_header.classes.values()[0]
    sensor.name = sensor_class['name']
    sensor.name_snake_case = convert_snake_case(sensor_class['name'])
    sensor.name_upper_case = convert_upper_case(sensor_class['name'])
    sensor_functions = sensor_class['methods']['public']
    
    sensor.functions = []
    for func in sensor_functions:
        deprecated = False
        function = Function()
        function.name = func['name']
        if function.name in [sensor.name, 'Load', 'Init']:
            continue

        function.returns = func['returns']
        only_parameters = []
        parameters = []
        print(function.name)
        for param in func['parameters']:
            if 'GAZEBO_DEPRECATED' in param['type']:
                deprecated = True
            only_parameters.append(param['name'])
            parameters.append(param['type'] + ' ' + param['name'])
        function.parameters = ', '.join(parameters)
        function.only_parameters = ', '.join(only_parameters)
        if not deprecated:
            sensor.functions.append(function)
    env = Environment(loader=FileSystemLoader(args.template_dir))
    template_binding = env.get_template('sensor_binding.tmpl')
    template_wrapper = env.get_template('sensor_wrapper.tmpl')

    with open(args.wrapper_path, 'w') as sensor_wrapper:
        sensor_wrapper.write(template_wrapper.render(sensor=sensor))

    with open(args.binding_path, 'w') as sensor_binding:
        sensor_binding.write(template_binding.render(sensor=sensor))
