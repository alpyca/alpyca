#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import time

from alpyca.launch import Launch, Master, Node, Runner


def main():
    launch = Launch.from_launch('alpyca_launch', 'test/basic.launch')

    with Master() as master:
        runner = Runner()
        runner.run(launch)


if __name__ == "__main__":
    main()
