#!/usr/bin/env python
PKG = 'alpyca_launch'

import sys
import unittest

from alpyca.launch import Launch, ParsingException


class TestArg(unittest.TestCase):

    def test_arg_basic(self):
        string = """<launch>
    <arg name="arg1" value="hello world" />
    <arg name="arg2" default="hello world" />
    <arg name="arg3" default="True" />
    <param name="arg1" value="$(arg arg1)" />
    <param name="arg2" value="$(arg arg2)" />
    <param name="arg3" type="bool" value="$(arg arg3)" />
</launch>"""

        launch = Launch.from_string(string)
        params = launch.args

        self.assertEqual(params['arg1'], 'hello world')
        self.assertEqual(params['arg2'], 'hello world')
        self.assertEqual(params['arg3'], True)

    def test_arg_from_external(self):

        args = {'arg1': 'test',
                'arg2': 'test'}
        string = """<launch>
    <arg name="arg1" />
    <arg name="arg2" default="hello world" />
    <param name="arg1" value="$(arg arg1)" />
    <param name="arg2" value="$(arg arg2)" />
</launch>"""

        launch = Launch.from_string(string, args=args)
        params = launch.args

        self.assertEqual(params['arg1'], 'test')
        self.assertEqual(params['arg2'], 'test')

    def test_arg_unset(self):
        string1 = """<launch>
    <arg name="arg1" />
    <param name="test" value="$(arg arg1)" />
</launch>"""

        with self.assertRaises(ParsingException):
            Launch.from_string(string1)

        string2 = """<launch>
    <arg name="arg1" />
    <param name="test" value="$(arg arg1)" />
</launch>"""

        with self.assertRaises(ParsingException):
            Launch.from_string(string2)

    def test_arg_whitespace(self):
        string = """<launch>
    <arg name="arg1" value="hello world" />
    <arg name="arg2" default="hello world" />
    <arg name="arg3" default="True" />
    <param name="arg1" value="$(arg arg1 )" />
    <param name="arg2" value="$(arg  arg2)" />
    <param name="arg3" type="bool" value="$(arg  arg3  )" />
</launch>"""

        launch = Launch.from_string(string)
        params = launch.args

        self.assertEqual(params['arg1'], 'hello world')
        self.assertEqual(params['arg2'], 'hello world')
        self.assertEqual(params['arg3'], True)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_arg_basic', TestArg)
    rostest.rosrun(PKG, 'test_arg_from_external', TestArg)
    rostest.rosrun(PKG, 'test_arg_unset', TestArg)
