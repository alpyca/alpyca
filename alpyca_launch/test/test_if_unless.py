#!/usr/bin/env python
PKG = 'alpyca_launch'

import unittest

from alpyca.launch import Launch, ParsingException


class TestIfUnless(unittest.TestCase):

    def test_if_unless_basic(self):
        string = """<launch>
    <param if="false" name="test_if" value="hello" />
    <param if="true" name="test_if" value="world" />
    <param if="1" name="test_if_number" value="hello" />
    <param if="0" name="test_if_number" value="world" />
    <param unless="false" name="test_unless" value="hello" />
    <param unless="true" name="test_unless" value="world" />
</launch>"""
        launch = Launch.from_string(string)

        params = launch.get_all_params()

        self.assertEqual(params['/test_if'].value, 'world')
        self.assertEqual(params['/test_if_number'].value, 'hello')
        self.assertEqual(params['/test_unless'].value, 'hello')

    def test_if_unless_invalid(self):
        string = """<launch><param if="true" unless="true" name="test" value="test" /></launch>"""
        with self.assertRaises(ParsingException):
	        Launch.from_string(string)

        string = """<launch><param if="unknown_value" name="test" value="test" /></launch>"""
        with self.assertRaises(ParsingException):
	        Launch.from_string(string)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_if_unless_basic', TestIfUnless)
    rostest.rosrun(PKG, 'test_if_unless_invalid', TestIfUnless)
