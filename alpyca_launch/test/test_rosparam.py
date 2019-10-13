#!/usr/bin/env python
PKG = 'alpyca_launch'

import math
import yaml
import unittest

from alpyca.launch import Launch, ParsingException


class TestRosparam(unittest.TestCase):

    def test_rosparam_basic(self):
        string = """<launch>
    <rosparam>
    test_ns:
        param1: true
        param2: hello
        param3: 3
        param4: 10.0
        param5: "3"
    </rosparam>
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/test_ns/param1'].value, bool))
        self.assertEqual(launch.params['/test_ns/param1'].value, True)
        self.assertTrue(isinstance(launch.params['/test_ns/param2'].value, str))
        self.assertEqual(launch.params['/test_ns/param2'].value, 'hello')
        self.assertTrue(isinstance(launch.params['/test_ns/param3'].value, int))
        self.assertEqual(launch.params['/test_ns/param3'].value, 3)
        self.assertTrue(isinstance(launch.params['/test_ns/param4'].value, float))
        self.assertAlmostEqual(launch.params['/test_ns/param4'].value, 10.0)
        self.assertTrue(isinstance(launch.params['/test_ns/param5'].value, str))
        self.assertEqual(launch.params['/test_ns/param5'].value, '3')

    def test_rosparam_emtpy(self):
        string = """<launch>
    <rosparam>
    </rosparam>

    <rosparam command="load" file="$(find alpyca_launch)/test/empty.yaml" />
</launch>"""
        launch = Launch.from_string(string)

    def test_rosparam_invalid_yaml(self):
        string = """<launch>
    <rosparam>
        hello: {{ invalid }} test
    </rosparam>
</launch>"""

        with self.assertRaises(yaml.parser.ParserError):
            launch = Launch.from_string(string)

    def test_rosparam_naming(self):
        string = """<launch>
    <rosparam param="a_list">[1, 2, 3, 4]</rosparam>

    <arg name="whitelist" default="[3, 2]"/>
    <rosparam param="whitelist" subst_value="True">$(arg whitelist)</rosparam>

    <rosparam ns="namespace">
        a: false
    </rosparam>
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/a_list'].value, list))
        self.assertEqual(len(launch.params['/a_list'].value), 4)
        self.assertTrue(isinstance(launch.params['/whitelist'].value, list))
        self.assertEqual(len(launch.params['/whitelist'].value), 2)
        self.assertTrue(isinstance(launch.params['/namespace/a'].value, bool))
        self.assertEqual(launch.params['/namespace/a'].value, False)

    def test_rosparam_explicit(self):
        string = """<launch>
    <rosparam>
    test_ns:
        param1: !!str 1
        param2: !!float 1
        param3: !!binary "aGVsbG8K"
    </rosparam>
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/test_ns/param1'].value, str))
        self.assertEqual(launch.params['/test_ns/param1'].value, '1')
        self.assertTrue(isinstance(launch.params['/test_ns/param2'].value, float))
        self.assertAlmostEqual(launch.params['/test_ns/param2'].value, 1.0)
        self.assertTrue(isinstance(launch.params['/test_ns/param3'].value, str))
        self.assertAlmostEqual(launch.params['/test_ns/param3'].value, 'hello\n')

    def test_rosparam_angle_extensions(self):
        string = """<launch>
    <rosparam>
    test_ns:
        param1: rad(2*pi)
        param2: deg(180)
        param3: !degrees 181.0
        param4: !radians 3.14169
    </rosparam>
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(isinstance(launch.params['/test_ns/param1'].value, float))
        self.assertAlmostEqual(launch.params['/test_ns/param1'].value, 2 * math.pi)
        self.assertTrue(isinstance(launch.params['/test_ns/param2'].value, float))
        self.assertAlmostEqual(launch.params['/test_ns/param2'].value, math.pi)
        self.assertTrue(isinstance(launch.params['/test_ns/param3'].value, float))
        self.assertAlmostEqual(launch.params['/test_ns/param3'].value, 181.0 * math.pi / 180.0)
        self.assertTrue(isinstance(launch.params['/test_ns/param4'].value, float))
        self.assertAlmostEqual(launch.params['/test_ns/param4'].value, 3.14169)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_rosparam_basic', TestRosparam)
    rostest.rosrun(PKG, 'test_rosparam_emtpy', TestRosparam)
    rostest.rosrun(PKG, 'test_rosparam_invalid_yaml', TestRosparam)
    rostest.rosrun(PKG, 'test_rosparam_naming', TestRosparam)
    rostest.rosrun(PKG, 'test_rosparam_explicit', TestRosparam)
    rostest.rosrun(PKG, 'test_rosparam_angle_extensions', TestRosparam)
