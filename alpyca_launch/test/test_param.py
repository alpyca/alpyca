#!/usr/bin/env python
PKG = 'alpyca_launch'

import yaml
import unittest

from alpyca.launch import Launch, ParsingException


class TestParam(unittest.TestCase):

    def test_global_param(self):
        string = """<launch>
    <param name="global_param" value="hello_world" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertEqual(launch.params['/global_param'].value, 'hello_world')

    def test_param_types(self):
        string = """<launch>
    <param name="int_param_auto" value="0" />
    <param name="int_param_forced" value="0" type="int" />

    <param name="double_param_auto" value="0.0" />
    <param name="double_param_forced" value="0" type="double" />

    <param name="str_param_auto" value="hello" />
    <param name="str_param_forced" value="0" type="str" />

    <param name="bool_param_auto" value="true" />
    <param name="bool_param_forced" value="true" type="boolean" />

    <param name="bool_param_auto_nonlowercase" value="True" />
    <param name="bool_param_forced_nonlowercase" value="True" type="boolean" />

    <param name="yaml_param" type="yaml" value="test_param: true" />
    <param name="yaml_param_scalar" type="yaml" value="true" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/int_param_auto'].value, int))
        self.assertEqual(launch.params['/int_param_auto'].value, 0)
        self.assertTrue(isinstance(launch.params['/int_param_forced'].value, int))
        self.assertEqual(launch.params['/int_param_forced'].value, 0)

        self.assertTrue(isinstance(launch.params['/double_param_auto'].value, float))
        self.assertAlmostEqual(launch.params['/double_param_auto'].value, 0.0)
        self.assertTrue(isinstance(launch.params['/double_param_forced'].value, float))
        self.assertAlmostEqual(launch.params['/double_param_forced'].value, 0.0)

        self.assertTrue(isinstance(launch.params['/str_param_auto'].value, str))
        self.assertEqual(launch.params['/str_param_auto'].value, 'hello')
        self.assertTrue(isinstance(launch.params['/str_param_forced'].value, str))
        self.assertEqual(launch.params['/str_param_forced'].value, '0')

        self.assertTrue(isinstance(launch.params['/bool_param_auto'].value, bool))
        self.assertEqual(launch.params['/bool_param_auto'].value, True)
        self.assertTrue(isinstance(launch.params['/bool_param_forced'].value, bool))
        self.assertEqual(launch.params['/bool_param_forced'].value, True)

        self.assertTrue(isinstance(launch.params['/bool_param_auto_nonlowercase'].value, bool))
        self.assertEqual(launch.params['/bool_param_auto_nonlowercase'].value, True)
        self.assertTrue(isinstance(launch.params['/bool_param_forced_nonlowercase'].value, bool))
        self.assertEqual(launch.params['/bool_param_forced_nonlowercase'].value, True)

        self.assertTrue(isinstance(launch.params['/yaml_param/test_param'].value, bool))
        self.assertEqual(launch.params['/yaml_param/test_param'].value, True)
        self.assertTrue(isinstance(launch.params['/yaml_param_scalar'].value, bool))
        self.assertEqual(launch.params['/yaml_param_scalar'].value, True)

    def test_param_command(self):
        string = """<launch>
    <param name="test" command="echo -n hello_world" />

    <param name="multiline" command="/bin/echo -ne hello\\nworld" />

    <param name="yaml_param" type="yaml" command="echo test_param: true" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/test'].value, str))
        self.assertEqual(launch.params['/test'].value, 'hello_world')
        self.assertTrue(isinstance(launch.params['/multiline'].value, str))
        self.assertEqual(launch.params['/multiline'].value, 'hello\nworld')

    def test_param_failing_command(self):
        string = """<launch><param name="test" command="false" /></launch>"""

        with self.assertRaises(ParsingException):
            launch = Launch.from_string(string)

    def test_textfile(self):
        string = """<launch>
    <param name="test" textfile="$(find alpyca_launch)/test/textfile.txt" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/test'].value, str))
        self.assertEqual(launch.params['/test'].value, 'hello_world')

    def test_textfile_does_not_exist(self):
        string = """<launch>
    <param name="test" textfile="$(find alpyca_launch)/test/textfile_does_not_exist.txt" />
</launch>"""

        with self.assertRaises(IOError):
            launch = Launch.from_string(string)

    def test_param_binfile(self):
        string = """<launch>
    <param name="test" binfile="$(find alpyca_launch)/test/textfile.txt" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/test'].value, str))
        self.assertEqual(launch.params['/test'].value, 'hello_world')

    def test_scoped_params(self):
        string = """<launch>
    <param name="global/param" value="abc" />
    <param name="/global/param2" value="def" />

    <group ns="namespace">
        <param name="test" value="val1" />
        <param name="/test2" value="val2" />
    </group>

    <node name="test_node" pkg="alpyca_launch" type="abort">
        <param name="private" value="val3" />
        <param name="~private2" value="val4" />
        <param name="/leading_slash" value="val5" />
    </node>
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/global/param'].value, str))
        self.assertEqual(launch.params['/global/param'].value, 'abc')
        self.assertTrue(isinstance(launch.params['/global/param2'].value, str))
        self.assertEqual(launch.params['/global/param2'].value, 'def')

        self.assertTrue(isinstance(launch.params['/namespace/test'].value, str))
        self.assertEqual(launch.params['/namespace/test'].value, 'val1')
        self.assertTrue(isinstance(launch.params['/test2'].value, str))
        self.assertEqual(launch.params['/test2'].value, 'val2')

        self.assertTrue(isinstance(launch.params['/test_node/private'].value, str))
        self.assertEqual(launch.params['/test_node/private'].value, 'val3')
        self.assertTrue(isinstance(launch.params['/test_node/private2'].value, str))
        self.assertEqual(launch.params['/test_node/private2'].value, 'val4')
        self.assertTrue(isinstance(launch.params['/test_node/leading_slash'].value, str))
        self.assertEqual(launch.params['/test_node/leading_slash'].value, 'val5')

    def test_scoped_params_with_double_slash(self):
        string = """<launch>
    <group ns="/">
        <param name="param1" value="hello" />
    </group>

    <node name="test_node" pkg="alpyca_launch" type="abort.py" ns="/racecar">
        <param name="private_param" value="hello again" />
    </node>
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/param1'].value, str))
        self.assertEqual(launch.params['/param1'].value, 'hello')
        self.assertTrue(isinstance(launch.params['/racecar/test_node/private_param'].value, str))
        self.assertEqual(launch.params['/racecar/test_node/private_param'].value, 'hello again')

    def test_wrong_param_types(self):
        with self.assertRaises(ValueError):
            string = """<launch><param name="test" value="abc" type="int" /></launch>"""
            launch = Launch.from_string(string)

        with self.assertRaises(ValueError):
            string = """<launch><param name="test" value="0.5" type="int" /></launch>"""
            launch = Launch.from_string(string)

        with self.assertRaises(ValueError):
            string = """<launch><param name="test" value="0.5" type="bool" /></launch>"""
            launch = Launch.from_string(string)

        with self.assertRaises(yaml.parser.ParserError):
            string = """<launch><param name="test" value="invalid: {{ yaml}} here" type="yaml" /></launch>"""
            launch = Launch.from_string(string)

        with self.assertRaises(yaml.parser.ParserError):
            string = """<launch><param name="test" command="echo -ne invalid: {{ yaml}} here" type="yaml" /></launch>"""
            launch = Launch.from_string(string)


        with self.assertRaises(ParsingException):
            string = """<launch><param name="test" value="0.5" type="unknown_type" /></launch>"""
            launch = Launch.from_string(string)

    def test_invalid_param_input_combinations(self):
        with self.assertRaises(ParsingException):
            string = """<launch><param name="test" value="abc" command="echo -ne test" /></launch>"""
            launch = Launch.from_string(string)

        with self.assertRaises(ParsingException):
            string = """<launch><param name="test" textfile="$(find alpyca_launch)/test/textfile.txt" command="echo -ne test" /></launch>"""
            launch = Launch.from_string(string)

        with self.assertRaises(ParsingException):
            string = """<launch><param name="test" /></launch>"""
            launch = Launch.from_string(string)

    def test_invalid_param_names(self):
        # TODO: Why is this name invalid?
        # with self.assertRaises(ParsingException):
        #     string = """<launch><param name="$%*" value="abc" /></launch>"""
        #     launch = Launch.from_string(string)

        with self.assertRaises(ParsingException):
            string = """<launch><param value="abc" /></launch>"""
            launch = Launch.from_string(string)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_global_param', TestParam)
    rostest.rosrun(PKG, 'test_param_types', TestParam)
    rostest.rosrun(PKG, 'test_param_command', TestParam)
    rostest.rosrun(PKG, 'test_param_failing_command', TestParam)
    rostest.rosrun(PKG, 'test_textfile', TestParam)
    rostest.rosrun(PKG, 'test_textfile_does_not_exist', TestParam)
    rostest.rosrun(PKG, 'test_param_binfile', TestParam)
    rostest.rosrun(PKG, 'test_scoped_params', TestParam)
    rostest.rosrun(PKG, 'test_scoped_params_with_double_slash', TestParam)
    rostest.rosrun(PKG, 'test_wrong_param_types', TestParam)
    rostest.rosrun(PKG, 'test_invalid_param_input_combinations', TestParam)
    rostest.rosrun(PKG, 'test_invalid_param_names', TestParam)
