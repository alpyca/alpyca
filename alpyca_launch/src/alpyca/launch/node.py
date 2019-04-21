
#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

from subprocess import Popen


__all__ = ['Node']

class Node(object):

    def __init__(self, package_name, executable, node_name):
        self.package_name = package_name
        self.executable = executable
        self.node_name = node_name

    def start(self):
        self.process = Popen(['rosrun', self.package_name, self.executable, '__name:=' + self.node_name])

    def stop(self):
        self.process.terminate()

    def __del__(self):
        self.process.terminate()
