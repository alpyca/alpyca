
#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import os
from subprocess import Popen, check_output


__all__ = ['Node']

class Node(object):

    def __init__(self, package_name, executable, node_name, ns=None, params=None, remaps=None, envs=None, extra_args=None, respawn=False, respawn_delay=0.0, required=False, clear_params=False, working_directory=None, launch_prefix=None):
        self.package_name = package_name
        self.executable = executable
        self.node_name = node_name
        self.ns = ns

        if params is None:
            self.params = {}
        else:
            self.params = params

        if remaps is None:
            self.remaps = {}
        else:
            self.remaps = remaps

        if envs is None:
            self.envs = {}
        else:
            self.envs = envs

        if extra_args is None:
            self.extra_args = {}
        else:
            self.extra_args = extra_args

        self.respawn = respawn
        self.respawn_delay = respawn_delay
        self.required = required
        self.clear_params = clear_params
        self.working_directory = working_directory

        if launch_prefix is None:
            self.launch_prefix = []
        else:
            self.launch_prefix = launch_prefix

        if ns is not None:
            self.full_name = ns + '/' + node_name
        else:
            self.full_name = node_name

            if self.full_name[0] != '/':
                self.full_name = '/' + self.full_name

    def start(self):
        self.running = True

        if self.ns is None:
            full_name = self.node_name
        else:
            full_name = self.ns + '/' + self.node_name
        
        if self.clear_params:
            check_output(['rosparam', 'delete', full_name])

        remaps_list = []
        params_list = []
        for key, value in self.remaps.items():
            remaps_list.append(key + ':=' + value)
        for key, value in self.params.items():
            params_list.append('_' + key.replace('~', '') + ':=' + value)

        env = os.environ.copy()
        env.update(self.envs)
        if self.respawn:
            while running:
                self.process = Popen(self.launch_prefix + ['rosrun', self.package_name, self.executable, '__name:=' + full_name] + remaps_list + params_list, env=env, cwd=self.working_directory)
                time.sleep(self.respawn_delay)
        else:
            self.process = Popen(self.launch_prefix + ['rosrun', self.package_name, self.executable, '__name:=' + full_name] + remaps_list + params_list, env=env, cwd=self.working_directory)
            if self.running and self.required:
                raise RuntimeError('Required node {} died!'.format(self.node_name))
    def stop(self):
        self.running = False
        self.process.terminate()

    def __del__(self):
        self.process.terminate()
