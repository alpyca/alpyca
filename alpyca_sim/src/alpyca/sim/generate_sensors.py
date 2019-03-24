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
    parser.add_argument('sensor_binding_path', type=str,
                        help='Output pybind11 bindings.')
    parser.add_argument('wrapper_dir', type=str,
                        help='Output directory for wrapper files.')
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


def inherits_sensor(sensor_class):
    found = False
    for inherits in sensor_class['inherits']:
        if inherits['class'] == 'Sensor':
            found = True
    return found

if __name__ == '__main__':
    args = get_args()
    gazebo_dir = find_gazebo(args.gazebo_include_dirs)
    class Sensor:
        pass
    class Function:
        pass
    class Parameter:
        pass

    sensors = []
    sensors_dir = os.path.join(gazebo_dir, 'sensors')
    for sensor_path in os.listdir(sensors_dir):
        if not sensor_path.endswith('.hh'):
            continue
        sensor = Sensor()
        sensor_file_name = os.path.basename(sensor_path).split('.')[0]

        sensor_abs_path = os.path.join(sensors_dir, sensor_path)

        try:
            cpp_header = CppHeaderParser.CppHeader(sensor_abs_path)
        except CppHeaderParser.CppHeaderParser.CppParseError:
            continue
            
        if not cpp_header.classes:
            continue

        sensor_class = cpp_header.classes.values()[0]

        if not inherits_sensor(sensor_class):
            continue 
        if sensor_class['name'] != 'ContactSensor':
            continue
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

        wrapper_file_name = convert_snake_case(sensor_file_name) + '_wrapper.h'
        sensor_wrapper_path = os.path.join(args.wrapper_dir, wrapper_file_name)
        with open(sensor_wrapper_path, 'w') as sensor_wrapper:
            sensor_wrapper.write(template_wrapper.render(sensor=sensor))

        sensors.append(sensor)
    
    with open(args.sensor_binding_path, 'w') as sensor_binding:
        sensor_binding.write(template_binding.render(sensors=sensors))
