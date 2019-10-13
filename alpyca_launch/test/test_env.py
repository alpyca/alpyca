#!/usr/bin/env python
PKG = 'alpyca_launch'

import unittest

from alpyca.launch import Launch, ParsingException


class TestEnv(unittest.TestCase):

    def test_env_basic(self):
        string = """<launch>
    <env name="test" value="hello world" />
    <node name="test_node" pkg="alpyca_launch" type="abort.py" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertEqual(len(launch.nodes), 1)

        node = launch.nodes['/test_node']
        envs = node.envs
        self.assertEqual(envs['test'], 'hello world')



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_env_basic', TestEnv)
