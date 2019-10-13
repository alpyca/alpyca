#!/usr/bin/env python
PKG = 'alpyca_launch'

import unittest

from alpyca.launch import Launch, ParsingException


class TestBasic(unittest.TestCase):

    def test_basic(self):
        string = """<launch>
</launch>"""
        Launch.from_string(string)

    def test_basic_invalid_xml(self):
        string = """<launch><abc>
</launch>"""
        with self.assertRaises(ParsingException):
            Launch.from_string(string)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_basic', TestBasic)
    rostest.rosrun(PKG, 'test_basic_invalid_xml', TestBasic)
