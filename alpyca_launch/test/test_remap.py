#!/usr/bin/env python
PKG = 'alpyca_launch'

import unittest

from alpyca.launch import Launch, ParsingException


class TestRemap(unittest.TestCase):

    def test_remap(self):
        string = """<launch>
    <remap from="topic_a" to="topic_b" />
    <node name="test" pkg="alpyca_launch" type="abort.py">
        <remap from="~local_a" to="/global_a" />
        <remap from="local_b" to="/global_b" />
    </node>

    <remap from="topic_c" to="topic_d" />
</launch>"""
        launch = Launch.from_string(string)
        node = launch.nodes['/test']

        self.assertEqual(node.remaps['topic_a'], 'topic_b')
        self.assertEqual(node.remaps['~local_a'], '/global_a')
        self.assertEqual(node.remaps['local_b'], '/global_b')

        self.assertNotIn('topic_c', node.remaps)

    def test_remap_scoped(self):
        string = """<launch>
    <group ns="namespace">
        <remap from="topic_a" to="topic_b" />
    </group>

    <node name="test" pkg="alpyca_launch" type="abort.py">
    </node>
</launch>"""
        launch = Launch.from_string(string)
        node = launch.nodes['/test']

        self.assertNotIn('topic_a', node.remaps)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_remap', TestRemap)
    rostest.rosrun(PKG, 'test_remap_scoped', TestRemap)
