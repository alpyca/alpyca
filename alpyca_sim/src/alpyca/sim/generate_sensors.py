#!/usr/bin/env python

import argparse
import os
import re

import CppHeaderParser
from jinja2 import Environment, FileSystemLoader


blacklist = set(['CameraSensor', 'MultiCameraSensor', 'RFIDSensor', 'GpuRaySensor'])

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
    parser.add_argument('gazebo_include_dir', type=str,
                        help='Include directory of gazebo. This directories is searched for the header files, for which bindings should be created.')
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


class Sensor:
    def __init__(self, header_sensor):
        self.name = header_sensor['name']
        self.name_snake_case = convert_snake_case(header_sensor['name'])
        self.name_upper_case = convert_upper_case(header_sensor['name'])
        sensor_functions = header_sensor['methods']['public']
        self.inherits = [inherits['class'] for inherits in header_sensor['inherits']]
        self.inherits_sensor = False

        self.functions = []
        for func in sensor_functions:
            function = Function(func)

            if function.name in [self.name, 'Load', 'Init']:
                continue
            if not function.deprecated:
                self.functions.append(function)


class Function:
    def __init__(self, header_func):
        deprecated = False
        self.name = header_func['name']
        self.returns = header_func['returns']
        only_parameters = []
        parameters = []

        for param in header_func['parameters']:
            if 'GAZEBO_DEPRECATED' in param['type']:
                deprecated = True
            only_parameters.append(param['name'])
            parameters.append(param['type'] + ' ' + param['name'])
        self.type_and_parameters = ', '.join(parameters)
        self.parameters = ', '.join(only_parameters)
        self.deprecated = deprecated


def filter_sensors(sensors):

    something_changed = True
    inherits_from_sensor = set()
    number_sensors = -1
    while len(inherits_from_sensor) != number_sensors:
        number_sensors = len(inherits_from_sensor)
        for sensor, _ in sensors:
 
            for inherits in sensor.inherits:
                if inherits == 'Sensor' or inherits in inherits_from_sensor:
                    inherits_from_sensor.add(sensor.name)

    return [sensor_tupple for sensor_tupple in sensors if sensor_tupple[0].name in inherits_from_sensor]


def read_sensors(gazebo_dir, wrapper_dir):
    possible_sensors = []
    sensors_dir = os.path.join(gazebo_dir, 'gazebo', 'sensors')
    for sensor_path in os.listdir(sensors_dir):
        if not sensor_path.endswith('.hh'):
            continue
        
        sensor_abs_path = os.path.join(sensors_dir, sensor_path)

        try:
            cpp_header = CppHeaderParser.CppHeader(sensor_abs_path)
        except CppHeaderParser.CppHeaderParser.CppParseError:
            continue
            
        if not cpp_header.classes:
            continue

        sensor_class = cpp_header.classes.values()[0]

        if sensor_class['name'] in blacklist:
            continue
        
        sensor = Sensor(sensor_class)

        sensor_file_name = os.path.basename(sensor_path).split('.')[0]
        wrapper_file_name = convert_snake_case(sensor_file_name) + '_wrapper.h'
        sensor_wrapper_path = os.path.join(wrapper_dir, wrapper_file_name)

        possible_sensors.append((sensor, sensor_wrapper_path))
    sensors = filter_sensors(possible_sensors)
    
    return sensors


def create_bindings(sensors, template_dir, sensor_binding_path):
    env = Environment(loader=FileSystemLoader(template_dir))
    template_binding = env.get_template('sensor_binding.tmpl')
    template_wrapper = env.get_template('sensor_wrapper.tmpl')

    for sensor, sensor_wrapper_path in sensors:
        with open(sensor_wrapper_path, 'w') as sensor_wrapper:
            sensor_wrapper.write(template_wrapper.render(sensor=sensor))

    with open(sensor_binding_path, 'w') as sensor_binding:
        sensor_binding.write(template_binding.render(sensors=sensors))


def main():
    args = get_args()
    sensors = read_sensors(args.gazebo_include_dir, args.wrapper_dir)
    create_bindings(sensors, args.template_dir, args.sensor_binding_path)


if __name__ == '__main__':
    main()
