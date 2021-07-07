# Copyright 2020 AIT Austrian Institute of Technology
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

import contextlib
from pathlib import Path
import os
import re
import sys
import tempfile
import time

import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
from launch_testing.asserts import EXIT_OK

import pytest


RESOURCE_PATH = Path(__file__).parent / 'resources'
OUTPUT_WAIT_TIMEOUT = 2
SHUTDOWN_TIMEOUT = 1


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestRos2BagConvert(unittest.TestCase):

    @classmethod
    def setUpClass(cls, launch_service, proc_info, proc_output):
        @contextlib.contextmanager
        def launch_bag_command(self, arguments, **kwargs):
            pkg_command_action = ExecuteProcess(
                cmd=['ros2', 'bag', 'convert', *arguments],
                additional_env={'PYTHONUNBUFFERED': '1'},
                name='ros2bag-cli',
                output='screen',
                **kwargs
            )
            with launch_testing.tools.launch_process(
                    launch_service, pkg_command_action, proc_info, proc_output
            ) as pkg_command:
                yield pkg_command
        cls.launch_bag_command = launch_bag_command
        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.allowed_exit_codes = [EXIT_OK, 1, 2]

    @classmethod
    def tearDownClass(cls):
        try:
            cls.tmpdir.cleanup()
        except OSError:
            if sys.platform != 'win32':
                raise
            # HACK to allow Windows to close pending file handles
            time.sleep(3)
            cls.tmpdir.cleanup()

    def test_empty(self):
        inbag_path = RESOURCE_PATH / 'empty_bag'
        outbag_path = Path(self.tmpdir.name) / 'ros2bag_convert'
        arguments = [inbag_path.as_posix(),
                     '--output', outbag_path.as_posix()]
        expected_in_output_regex = re.compile(
            r'\[rosbag2_storage]: Opened database .* for READ_ONLY')
        expected_out_output_regex = re.compile(
            r'\[rosbag2_storage]: Opened database .* for READ_WRITE')
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_output(
                condition=lambda output: expected_in_output_regex.search(output) is not None,
                timeout=OUTPUT_WAIT_TIMEOUT)
            bag_command.wait_for_output(
                condition=lambda output: expected_out_output_regex.search(output) is not None,
                timeout=OUTPUT_WAIT_TIMEOUT)
        bag_command.wait_for_shutdown(timeout=SHUTDOWN_TIMEOUT)
        assert bag_command.terminated
        assert bag_command.exit_code in self.allowed_exit_codes

    def test_compressed_output(self):
        inbag_path = RESOURCE_PATH / 'empty_bag'
        outbag_path = Path(self.tmpdir.name) / 'ros2bag_convert'
        arguments = [inbag_path.as_posix(),
                     '--output', outbag_path.as_posix(),
                     '--compression-mode', 'file',
                     '--compression-format', 'zstd']
        expected_in_output_regex = re.compile(
            r'\[rosbag2_storage]: Opened database .* for READ_ONLY')
        expected_out_output_regex = re.compile(
            r'\[rosbag2_storage]: Opened database .* for READ_WRITE')
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_output(
                condition=lambda output: expected_in_output_regex.search(output) is not None,
                timeout=OUTPUT_WAIT_TIMEOUT)
            bag_command.wait_for_output(
                condition=lambda output: expected_out_output_regex.search(output) is not None,
                timeout=OUTPUT_WAIT_TIMEOUT)
        bag_command.wait_for_shutdown(timeout=SHUTDOWN_TIMEOUT)
        assert bag_command.terminated
        assert bag_command.exit_code in self.allowed_exit_codes
