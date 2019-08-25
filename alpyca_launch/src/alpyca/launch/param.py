
#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

from subprocess import Popen


__all__ = ['Param']

class Param(object):

    def __init__(self, name, value):
        self.name = name
        self.value = value

    @classmethod
    def from_xml_element(cls, element):
        # TODO
        pass

    @staticmethod
    def from_dicts(collection, ns=''):
        parameters = []
        if isinstance(collection, dict):
            for param in [Param.from_dicts(value, ns + '/' + key) for key, value in collection.items()]:
                parameters += param
        elif ns != '':
            parameters.append(Param(ns, collection))

        return parameters

    def start(self):
        self.process = Popen(['rosparam', 'set', self.name, str(self.value)])

    def __del__(self):
        self.process.terminate()
