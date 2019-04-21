#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import time

from alpyca.launch import Launch, Master, Node, Runner


def main():
    sub_launch = Launch()
    sub_launch.add_node(Node('turtlesim', 'turtlesim_node', 'node1'))

    main_launch = Launch()
    main_launch.add_node(Node('turtlesim', 'turtlesim_node', 'node2'))
    main_launch.add_launch(sub_launch)

    with Master() as master:
        runner = Runner()
        runner.run(main_launch)


if __name__ == "__main__":
    main()
