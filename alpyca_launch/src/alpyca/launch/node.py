
#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import os
import time
from subprocess import Popen, check_output, PIPE
from threading  import Thread

try:
    from queue import Queue, Empty
except ImportError:
    from Queue import Queue, Empty  # python 2.x


__all__ = ['Node']

class Node(object):

    def __init__(self, package_name, executable, node_name, ns=None, params=None, remaps=None, envs=None, extra_args=None, respawn=False, respawn_delay=1.0, required=False, clear_params=False, working_directory=None, launch_prefix=None):
        self.package_name = package_name
        self.executable = executable
        self.node_name = node_name
        self.ns = ns

        self.running = False

        if params is None:
            self.params = {}
        else:
            self.params = params

        if remaps is None:
            self.remaps = {}
        else:
            self.remaps = remaps

        self.remaps_list = []
        self.params_list = []
        for key, value in self.remaps.items():
            self.remaps_list.append(key + ':=' + value)
        for key, value in self.params.items():
            self.params_list.append('_' + key.replace('~', '') + ':=' + value)

        if envs is None:
            self.envs = {}
        else:
            self.envs = envs

        if extra_args is None:
            self.extra_args = []
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

        self.env = os.environ.copy()
        self.env.update(self.envs)

        self.process = None

    def _run_process(self):
        # The output stream is line buffered with "stdbuf -o L"
        base_cmd = ['stdbuf', '-o', 'L', 'rosrun', self.package_name, self.executable, '__name:=' + self.node_name]
        if self.ns is not None:
            base_cmd.append('__ns:=' + self.ns)

        cmd = self.launch_prefix + base_cmd + self.remaps_list + self.params_list + self.extra_args

        self.process = Popen(cmd, env=self.env, cwd=self.working_directory, stdout=PIPE, stderr=PIPE)
        self.queue = Queue()
        self.output_thread = Thread(target=self._enqueue_output, args=(self.process.stdout, self.queue))
        self.output_thread.start()

        self.running = True
        return_code = self.process.wait()
        print('Node {} finished with return code {}'.format(self.full_name, return_code))
        if self.running and self.required:
            raise RuntimeError('Required node {} died!'.format(self.node_name))

    def start(self):
        print('Started node {}'.format(self.full_name))

        if self.clear_params:
            check_output(['rosparam', 'delete', self.full_name])

        if self.respawn:
            while self.running:
                self._run_process()
                time.sleep(self.respawn_delay)
        else:
            self._run_process()

    def stop(self):
        if self.running:
            try:
                self.process.terminate()
            except OSError:
                # Process has already finished
                pass
        self.running = False

    def __del__(self):
        self.stop()

    def _enqueue_output(self, out, queue):
        for line in iter(out.readline, b''):
            queue.put(line)
        out.close()

    def print_output(self):
        if self.process is not None:
            while True:
                try:  line = self.queue.get_nowait()
                except Empty:
                    return
                else:
                    print(self.full_name + ': \t' + line, end='')
