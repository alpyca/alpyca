#!/usr/bin/env python

import argparse
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
    parser.add_argument('template_dir')
    parser.add_argument('binding_path')
    parser.add_argument('wrapper_path')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = get_args()

    class Sensor:
        pass
    class Function:
        pass
    class Parameter:
        pass
    sensor = Sensor()
    cpp_header = CppHeaderParser.CppHeader(r'/usr/include/gazebo-7/gazebo/sensors/ContactSensor.hh')
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
