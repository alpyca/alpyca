#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import time

from alpyca.launch import Launch, Master, Node, Runner


def main():
    sub1_launch = Launch()
    sub1_launch.add_node(Node('turtlesim', 'turtlesim_node', 'node1'))

    sub2_launch = Launch.from_launch('alpyca_launch', 'example.launch')
    sub3_launch = Launch.from_launch('uuv_descriptions', 'upload_rexrov.launch')

    main_launch = Launch()
    main_launch.add_node(Node('turtlesim', 'turtlesim_node', 'node2'))
    main_launch.add_launch(sub1_launch)
    main_launch.add_launch(sub2_launch)
    main_launch.add_launch(sub3_launch)

    with Master() as master:
        runner = Runner()
        runner.run(main_launch)


if __name__ == "__main__":
    main()
