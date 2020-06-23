# Copyright 2020, Robotec.ai sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""System monitor for monitoring disk, memory and cpu utilization."""

import pathlib
import re
import signal
import subprocess
import threading

import psutil

import rclpy
from rclpy.node import Node


class SystemMonitor(Node):
    """ROS2 node for monitoring disk, memory and cpu utilization."""

    dstat_proc = None
    iotop_ready = False

    def __init__(self):
        """Initialize monitor."""
        super().__init__('system_monitor')
        self.logger = rclpy.logging.get_logger('SYSTEM_MONITOR')
        self.declare_parameter('benchmark_path')
        self.declare_parameter('frequency', 10)

        self.__benchmark_path = \
            self.get_parameter('benchmark_path') \
                .get_parameter_value().string_value
        self.__frequency = \
            self.get_parameter('frequency').get_parameter_value().integer_value

        if self.__frequency == 0:
            self.logger.warn(
                'Monitor frequency is set to 0. Invalid value. \
                Rolling back to 10Hz.'
            )
            self.__frequency = 10

        self.create_timer(1/self.__frequency, self.gather_cpu_and_mem_data)
        self.iotop_thread = threading.Thread(None, self.__launch_iotop)
        self.iotop_thread.start()

        bench_path = pathlib.Path(self.__benchmark_path)
        self.__cpu_file = open(bench_path.joinpath('system_cpu.csv'), 'w')
        self.__mem_file = open(bench_path.joinpath('system_mem.csv'), 'w')
        self.__disk_file = open(bench_path.joinpath('system_disk.csv'), 'w')

    def __launch_iotop(self):
        self.dstat_proc = subprocess.Popen(
            [
                'pkexec',
                'iotop',
                '-b',
                '-o',
                '-d',
                str(1/self.__frequency),
                '--kilobytes'
            ],
            stdout=subprocess.PIPE, universal_newlines=True
        )

        while rclpy.ok():
            line = self.dstat_proc.stdout.readline()
            if 'Total' in line:
                if not self.iotop_ready:
                    self.logger.info('Monitor ready.')
                    self.iotop_ready = True

                match = re.search(
                    (
                        r'Total DISK READ :\s+([0-9]+.[0-9]+).+ \| '
                        r'Total DISK WRITE :\s+([0-9]+.[0-9]+)'
                    ),
                    line
                )
                if match:
                    read = match.group(1)
                    write = match.group(2)
                    if not self.__disk_file.closed:
                        self.__disk_file.write(
                            str(self.get_clock().now().nanoseconds)
                            + ';' + read + ';' + write + '\n'
                        )

    def stop_iotop(self):
        """Stop iotop tool."""
        if self.dstat_proc:
            self.dstat_proc.kill()

    def close_files(self):
        """Close all open files."""
        self.__disk_file.close()
        self.__cpu_file.close()
        self.__mem_file.close()

    def gather_cpu_and_mem_data(self):
        """Gather cpu and memory utilization info."""
        if self.iotop_ready:
            cpu_percents = psutil.cpu_percent(interval=0, percpu=True)
            cpu_avg = psutil.cpu_percent(interval=0)
            mem = psutil.virtual_memory()
            cpu_percents_str = ';' + str(cpu_avg)
            for p in cpu_percents:
                cpu_percents_str += ';' + (str(p))
            if not self.__cpu_file.closed:
                self.__cpu_file.write(
                    str(self.get_clock().now().nanoseconds) +
                    cpu_percents_str + '\n')
            if not self.__mem_file.closed:
                self.__mem_file.write(
                    str(self.get_clock().now().nanoseconds) +
                    ';' + str(mem.used) + '\n')


def signal_handler(signal, frame):
    """Intercept signal for shuting down."""
    rclpy.shutdown()


def main():
    """Ros2 once-spin run."""
    rclpy.init()
    signal.signal(signal.SIGINT, signal_handler)
    node = SystemMonitor()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.close_files()
    node.destroy_node()
    exit(0)


if __name__ == '__main__':
    main()
