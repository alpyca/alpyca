#!/usr/bin/env python
PKG = 'alpyca_launch'

import unittest

from alpyca.launch import Launch, ParsingException


class TestInclude(unittest.TestCase):

    def test_include_basic(self):
        string = """<launch>
    <arg name="test_argument" value="hello" />
    <include file="$(find alpyca_launch)/test/basic_sub.launch">
        <arg name="test_argument" value="$(arg test_argument)" />
    </include>
</launch>"""
        launch = Launch.from_string(string)

        params = launch.get_all_params()

        self.assertEqual(params['/test_argument'].value, 'hello')

    def test_include_default(self):
        string = """<launch>
    <arg name="test_argument" value="hello" />
    <include file="$(find alpyca_launch)/test/basic_sub.launch">
        <arg name="test_argument" default="$(arg test_argument)" />
    </include>
</launch>"""
        launch = Launch.from_string(string)

        params = launch.get_all_params()

        self.assertEqual(params['/test_argument'].value, 'hello')

    def test_pass_all(self):
        string = """<launch>
    <arg name="test_argument" value="hello" />
    <include file="$(find alpyca_launch)/test/basic_sub.launch" pass_all_args="true" />
</launch>"""
        launch = Launch.from_string(string)

        params = launch.get_all_params()

        self.assertEqual(params['/test_argument'].value, 'hello')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_include_basic', TestInclude)
    rostest.rosrun(PKG, 'test_include_default', TestInclude)
    rostest.rosrun(PKG, 'test_pass_all', TestInclude)
