#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

from subprocess import Popen
import time


__all__ = ['Master']

class Master(object):

    def __init__(self):
        pass

    def wait_for_running(self):
        # TODO: Implement properly
        time.sleep(2)

    def __enter__(self):
        print('start roscore')
        self.process = Popen(['roscore'])
        self.wait_for_running()
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.process.terminate()
