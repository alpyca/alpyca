#!/usr/bin/env python
PKG = 'alpyca_launch'

import unittest

from alpyca.launch import Launch, ParsingException


class TestNode(unittest.TestCase):

    def test_node_basic(self):
        string = """<launch>
    <node name="test_node" pkg="alpyca_launch" type="abort.py" />
</launch>"""
        launch = Launch.from_string(string)

        node = launch.nodes['/test_node']

        self.assertEqual(node.node_name, 'test_node')
        self.assertEqual(node.package_name, 'alpyca_launch')
        self.assertEqual(node.executable, 'abort.py')

    def test_node_invalid(self):
        string = """<launch>
    <node name="test_node" />
</launch>"""
        with self.assertRaises(ParsingException):
            launch = Launch.from_string(string)

    def test_node_arg(self):
        string = """<launch>
    <node name="test_node" pkg="alpyca_launch" type="abort.py" args="arg1 arg2 'long arg'" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertEqual(len(launch.nodes), 1)

        node = launch.nodes['/test_node']
        args = node.extra_args
        self.assertEqual(len(args), 3)
        self.assertEqual(args[0], 'arg1')
        self.assertEqual(args[1], 'arg2')
        self.assertEqual(args[2], 'long arg')

    def test_node_respawn(self):
        string = """<launch>
    <node name="test_node" pkg="rosmon_core" type="abort" respawn="true" respawn_delay="10" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertEqual(len(launch.nodes), 1)

        node = launch.nodes['/test_node']

        self.assertTrue(node.respawn)
        self.assertAlmostEqual(node.respawn_delay, 10.0)

    def test_node_required(self):
        string = """<launch>
    <node name="test_node" pkg="alpyca_launch" type="abort.py" required="true" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertEqual(len(launch.nodes), 1)

        node = launch.nodes['/test_node']

        self.assertTrue(node.required)

    def test_node_ns(self):
        string = """<launch>
    <node name="test_node" pkg="alpyca_launch" type="abort.py" ns="namespace" />
    <group ns="ns1">
        <node name="test_node" pkg="alpyca_launch" type="abort.py" />
        <node name="test_node" pkg="alpyca_launch" type="abort.py" ns="namespace" />
    </group>
</launch>"""
        launch = Launch.from_string(string)

        self.assertEqual(len(launch.nodes), 3)

        self.assertEqual(launch.nodes['/namespace/test_node'].ns, '/namespace')
        self.assertEqual(launch.nodes['/ns1/test_node'].ns, '/ns1')
        self.assertEqual(launch.nodes['/ns1/namespace/test_node'].ns, '/ns1/namespace')

    def test_node_clear_params(self):
        string = """<launch>
    <node name="test_node_on" pkg="alpyca_launch" type="abort.py" clear_params="true" />
    <node name="test_node_off" pkg="alpyca_launch" type="abort.py" clear_params="false" />
    <node name="test_node_def" pkg="alpyca_launch" type="abort.py" />
</launch>"""
        launch = Launch.from_string(string)

        self.assertTrue(launch.nodes['/test_node_on'].clear_params)
        self.assertFalse(launch.nodes['/test_node_off'].clear_params)
        self.assertFalse(launch.nodes['/test_node_def'].clear_params)

    def test_node_cwd(self):
        string = """<launch>
    <node name="test_node" pkg="alpyca_launch" type="abort.py" cwd="/my_cwd/" />
</launch>"""
        launch = Launch.from_string(string)
        node = launch.nodes['/test_node']

        self.assertEqual(node.working_directory, '/my_cwd/')

    def test_node_launch_prefix(self):
        string = """<launch>
    <node name="test_node" pkg="rosmon_core" type="abort" launch-prefix="echo my command is:" />
</launch>"""
        launch = Launch.from_string(string)
        node = launch.nodes['/test_node']

        prefix = node.launch_prefix

        self.assertEqual(prefix[0], 'echo')
        self.assertEqual(prefix[1], 'my')
        self.assertEqual(prefix[2], 'command')
        self.assertEqual(prefix[3], 'is:')

    def test_node_remap(self):
        string = """<launch>
    <node name="test_node" pkg="alpyca_launch" type="abort.py">
        <remap from="private1" to="/global_target" />
        <remap from="private2" to="local_target" />
    </node>
</launch>"""
        launch = Launch.from_string(string)
        node = launch.nodes['/test_node']

        self.assertEqual(len(node.remaps), 2)
        self.assertEqual(node.remaps['private1'], '/global_target')
        self.assertEqual(node.remaps['private2'], 'local_target')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_node_basic', TestNode)
    rostest.rosrun(PKG, 'test_node_invalid', TestNode)
    rostest.rosrun(PKG, 'test_node_arg', TestNode)
    rostest.rosrun(PKG, 'test_node_respawn', TestNode)
    rostest.rosrun(PKG, 'test_node_required', TestNode)
    rostest.rosrun(PKG, 'test_node_ns', TestNode)
    rostest.rosrun(PKG, 'test_node_clear_params', TestNode)
    rostest.rosrun(PKG, 'test_node_cwd', TestNode)
    rostest.rosrun(PKG, 'test_node_launch_prefix', TestNode)
    rostest.rosrun(PKG, 'test_node_remap', TestNode)
