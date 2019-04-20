#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import time

from alpyca.launch import Launcher, Master, Node


def main():
    with Master() as master:
        sub_launcher = Launcher()
        sub_launcher.add_node(Node('turtlesim', 'turtlesim_node', 'node1'))

        main_launcher = Launcher()
        main_launcher.add_node(Node('turtlesim', 'turtlesim_node', 'node2'))
        main_launcher.add_launcher(sub_launcher)
        main_launcher.run()

if __name__ == "__main__":
    main()
