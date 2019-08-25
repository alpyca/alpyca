
#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

from subprocess import Popen


__all__ = ['Node']

class Node(object):

    def __init__(self, package_name, executable, node_name, params=None, remaps=None):
        self.package_name = package_name
        self.executable = executable
        self.node_name = node_name

        if params is None:
            self.params = {}
        else:
            self.params = params

        if remaps is None:
            self.remaps = {}
        else:
            self.remaps = remaps

    @classmethod
    def from_xml_element(cls, element):
        package_name = element.get('pkg')
        executable = element.get('type')
        node_name = element.get('name')
        return cls(package_name, executable, node_name)

    def start(self):
        remaps_list = []
        params_list = []
        for key, value in self.remaps.items():
            remaps_list.append(key + ':=' + value)
        for key, value in self.params.items():
            params_list.append('_' + key.replace('~', '') + ':=' + value)

        self.process = Popen(['rosrun', self.package_name, self.executable, '__name:=' + self.node_name] + remaps_list + params_list)

    def stop(self):
        self.process.terminate()

    def __del__(self):
        self.process.terminate()
