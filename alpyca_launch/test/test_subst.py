#!/usr/bin/env python
PKG = 'alpyca_launch'

import os
import math
import yaml
import unittest
import subprocess

from alpyca.launch import Launch, ParsingException


def find_package(package_name):
    return subprocess.check_output(['rospack', 'find', package_name]).replace('\n', '')


class TestSubst(unittest.TestCase):

    def test_env_basic(self):
        string = """<launch>
    <param name="env_test" value="$(env PATH)" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/env_test'].value, str))
        self.assertEqual(launch.params['/env_test'].value, os.environ['PATH'])

    def test_env_failure(self):
        string = """<launch>
    <param name="env_test" value="$(env ALPYCA_UNLIKELY_TO_BE_SET)" />
</launch>"""

        with self.assertRaises(KeyError):
            launch = Launch.from_string(string)

    def test_optenv_present(self):
        string = """<launch>
    <param name="env_test" value="$(optenv PATH no_such_thing)" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/env_test'].value, str))
        self.assertEqual(launch.params['/env_test'].value, os.environ['PATH'])

    def test_optenv_not_present(self):
        string = """<launch>
    <param name="env_test" value="$(optenv ALPYCA_UNLIKELY_TO_BE_SET no_such_thing)" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/env_test'].value, str))
        self.assertEqual(launch.params['/env_test'].value, 'no_such_thing')

    def test_optenv_not_present_long(self):
        string = """<launch>
    <param name="env_test" value="$(optenv ALPYCA_UNLIKELY_TO_BE_SET no such thing)" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/env_test'].value, str))
        self.assertEqual(launch.params['/env_test'].value, 'no such thing')

    def test_find_regular_use(self):
        string = """<launch>
    <param name="path_to_alpyca" value="$(find alpyca_launch)" />
    <param name="path_to_launch_file" value="$(find alpyca_launch)/test/basic.launch" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/path_to_alpyca'].value, str))
        self.assertEqual(launch.params['/path_to_alpyca'].value, find_package('alpyca_launch'))
        self.assertTrue(isinstance(launch.params['/path_to_launch_file'].value, str))
        self.assertEqual(launch.params['/path_to_launch_file'].value, find_package('alpyca_launch') + '/test/basic.launch')

    def test_find_unknown_package(self):
        string = """<launch>
    <param name="test" value="$(find alpyca_this_package_is_unlikely_to_be_there)" />
</launch>"""
        with self.assertRaises(ParsingException):
            launch = Launch.from_string(string)

    def test_anon_regular_use(self):
        string = """<launch>
    <param name="test_1" value="$(anon rviz-1)" />
    <param name="test_2" value="$(anon rviz-1)" />
    <param name="test_3" value="$(anon rviz-2)" />
</launch>"""

        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/test_1'].value, str))
        self.assertTrue(isinstance(launch.params['/test_2'].value, str))
        self.assertTrue(isinstance(launch.params['/test_3'].value, str))

        self.assertEqual(launch.params['/test_1'].value, launch.params['/test_2'].value)
        self.assertNotEqual(launch.params['/test_1'].value, launch.params['/test_3'].value)

    def test_anon_clash_example(self):
        string = """<launch>
    <node name="$(anon foo)" pkg="rospy_tutorials" type="talker.py" />
    <node name="$(anon foo)" pkg="rospy_tutorials" type="talker.py" />
</launch>"""

        with self.assertRaises(ParsingException):
            launch = Launch.from_string(string)

    def test_arg_basic_use(self):
        string = """<launch>
    <arg name="test_arg" default="hello" />
    <param name="test" value="$(arg test_arg)" />
</launch>"""

        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/test'].value, str))
        self.assertEqual(launch.params['/test'].value, 'hello')

    def test_arg_unknown_arg(self):
        string = """<launch>
    <arg name="test_arg" default="hello" />
    <param name="test" value="$(arg test_arg_unknown)" />
</launch>"""

        with self.assertRaises(ParsingException):
            launch = Launch.from_string(string)

    def test_eval_example_1(self):
        string = """<launch>
    <arg name="radius" default="2.0" />
    <param name="circumference" value="$(eval 2.* pi * arg('radius'))"/>
</launch>"""

        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/circumference'].value, float))
        self.assertAlmostEqual(launch.params['/circumference'].value, 2 * math.pi * 2)

    def test_eval_example_2(self):
        string = """<launch>
    <arg name="foo" default="test" />
    <param name="test" value="$(eval arg('foo') + env('PATH') + 'bar' + find('alpyca_launch'))"/>
</launch>"""

        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/test'].value, str))
        self.assertEqual(launch.params['/test'].value, 'test' + os.environ['PATH'] + 'bar' + find_package('alpyca_launch'))

    def test_eval_example_3(self):
        string = """<launch>
    <arg name="foo" default="test" />
    <param name="test" value="$(eval foo + env('PATH') + 'bar' + find('alpyca_launch'))"/>
</launch>"""

        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/test'].value, str))
        self.assertEqual(launch.params['/test'].value, 'test' + os.environ['PATH'] + 'bar' + find_package('alpyca_launch'))

    def test_eval_example_4(self):
        string = """<launch>
    <arg name="foo" default="test" />
    <param name="test" value="$(eval 2 * 20)"/>
</launch>"""

        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/test'].value, int))
        self.assertEqual(launch.params['/test'].value, 2 * 20)

    def test_eval_python_error_1(self):
        string = """<launch>
    <arg name="foo" default="test" />
    <param name="test" value="$(eval )))"/>
</launch>"""

        with self.assertRaises(SyntaxError):
            launch = Launch.from_string(string)

    def test_eval_python_error_2(self):
        string = """<launch>
    <arg name="foo" default="test" />
    <param name="test" value="$(eval acos)"/>
</launch>>"""

        with self.assertRaises(ParsingException):
            launch = Launch.from_string(string)

    def test_eval_lowercase_booleans(self):
        string = """<launch>
    <arg name="input" default="false" />
    <param name="output" value="$(eval input == true)"/>
</launch>"""

        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/output'].value, bool))
        self.assertEqual(launch.params['/output'].value, False)

    def test_dirname(self):
        string = """<launch>
    <param name="test" value="$(dirname)" />
</launch>"""

        file_path = os.path.abspath(__file__)
        launch = Launch.from_string(string, file_path)

        self.assertTrue(isinstance(launch.params['/test'].value, str))
        self.assertEqual(launch.params['/test'].value, os.path.dirname(file_path))

    def test_subst_invalid(self):
        string = """<launch>
    <param name="test" value="$(unknown_subst parameter)" />
</launch>"""

        with self.assertRaises(ParsingException):
            launch = Launch.from_string(string)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_env_basic', TestSubst)
    rostest.rosrun(PKG, 'test_env_failure', TestSubst)
    rostest.rosrun(PKG, 'test_optenv_present', TestSubst)
    rostest.rosrun(PKG, 'test_optenv_not_present', TestSubst)
    rostest.rosrun(PKG, 'test_optenv_not_present_long', TestSubst)
    rostest.rosrun(PKG, 'test_find_regular_use', TestSubst)
    rostest.rosrun(PKG, 'test_unknown_package', TestSubst)
    rostest.rosrun(PKG, 'test_anon_regular_use', TestSubst)
    rostest.rosrun(PKG, 'test_anon_clash_example', TestSubst)
    rostest.rosrun(PKG, 'test_arg_basic_use', TestSubst)
    rostest.rosrun(PKG, 'test_arg_unknown_arg', TestSubst)
    rostest.rosrun(PKG, 'test_eval_example_1', TestSubst)
    rostest.rosrun(PKG, 'test_eval_example_2', TestSubst)
    rostest.rosrun(PKG, 'test_eval_example_3', TestSubst)
    rostest.rosrun(PKG, 'test_eval_example_4', TestSubst)
    rostest.rosrun(PKG, 'test_eval_python_error_1', TestSubst)
    rostest.rosrun(PKG, 'test_eval_python_error_2', TestSubst)
    rostest.rosrun(PKG, 'test_eval_lowercase_booleans', TestSubst)
    rostest.rosrun(PKG, 'test_dirname', TestSubst)
    rostest.rosrun(PKG, 'test_subst_invalid', TestSubst)
