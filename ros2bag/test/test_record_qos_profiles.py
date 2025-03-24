# Copyright 2020 Amazon.com, Inc. or its affiliates. All rights reserved.
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

import pytest


PROFILE_PATH = Path(__file__).parent / 'resources'
TEST_NODE = 'ros2bag_record_qos_profile_test_node'
TEST_NAMESPACE = 'ros2bag_record_qos_profile'
ERROR_STRING_MSG = 'ros2bag CLI did not produce the expected output'\
        '\n Expected output pattern: {}\n Actual output: {}'
OUTPUT_WAIT_TIMEOUT = 10
SHUTDOWN_TIMEOUT = 5


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestRos2BagRecord(unittest.TestCase):

    @classmethod
    def setUpClass(cls, launch_service, proc_info, proc_output):
        @contextlib.contextmanager
        def launch_bag_command(self, arguments, **kwargs):
            pkg_command_action = ExecuteProcess(
                cmd=['ros2', 'bag', *arguments],
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

    def test_qos_simple(self):
        profile_path = PROFILE_PATH / 'qos_profile.yaml'
        output_path = Path(self.tmpdir.name) / 'ros2bag_test_basic'
        arguments = ['record', '-a', '--qos-profile-overrides-path', profile_path.as_posix(),
                     '--output', output_path.as_posix()]
        expected_output = 'Listening for topics...'
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_output(
                condition=lambda output: expected_output in output,
                timeout=OUTPUT_WAIT_TIMEOUT)
        bag_command.wait_for_shutdown(timeout=SHUTDOWN_TIMEOUT)
        assert bag_command.terminated
        matches = expected_output in bag_command.output
        assert matches, ERROR_STRING_MSG.format(expected_output, bag_command.output)

    def test_incomplete_qos_profile(self):
        profile_path = PROFILE_PATH / 'incomplete_qos_profile.yaml'
        output_path = Path(self.tmpdir.name) / 'ros2bag_test_incomplete'
        arguments = ['record', '-a', '--qos-profile-overrides-path', profile_path.as_posix(),
                     '--output', output_path.as_posix()]
        expected_output = 'Listening for topics...'
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_output(
                condition=lambda output: expected_output in output,
                timeout=OUTPUT_WAIT_TIMEOUT)
        bag_command.wait_for_shutdown(timeout=SHUTDOWN_TIMEOUT)
        assert bag_command.terminated
        matches = expected_output in bag_command.output
        assert matches, ERROR_STRING_MSG.format(expected_output, bag_command.output)

    def test_incomplete_qos_duration(self):
        profile_path = PROFILE_PATH / 'incomplete_qos_duration.yaml'
        output_path = Path(self.tmpdir.name) / 'ros2bag_test_incomplete_duration'
        arguments = ['record', '-a', '--qos-profile-overrides-path', profile_path.as_posix(),
                     '--output', output_path.as_posix()]
        expected_string_regex = re.compile(
            r'\[ERROR] \[ros2bag]: Time overrides must include both')
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_output(
                condition=lambda output: expected_string_regex.search(output) is not None,
                timeout=OUTPUT_WAIT_TIMEOUT)
        bag_command.wait_for_shutdown(timeout=SHUTDOWN_TIMEOUT)
        assert bag_command.terminated
        assert bag_command.exit_code != launch_testing.asserts.EXIT_OK
        matches = expected_string_regex.search(bag_command.output)
        assert matches, ERROR_STRING_MSG.format(expected_string_regex.pattern, bag_command.output)

    def test_nonexistent_qos_profile(self):
        profile_path = PROFILE_PATH / 'foobar.yaml'
        output_path = Path(self.tmpdir.name) / 'ros2bag_test_nonexistent'
        arguments = ['record', '-a', '--qos-profile-overrides-path', profile_path.as_posix(),
                     '--output', output_path.as_posix()]
        expected_string_regex = re.compile(
            r'ros2 bag record: error: argument --qos-profile-overrides-path: can\'t open')
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_output(
                condition=lambda output: expected_string_regex.search(output) is not None,
                timeout=OUTPUT_WAIT_TIMEOUT)
        bag_command.wait_for_shutdown(timeout=SHUTDOWN_TIMEOUT)
        assert bag_command.terminated
        assert bag_command.exit_code != launch_testing.asserts.EXIT_OK
        matches = expected_string_regex.search(bag_command.output)
        assert matches, ERROR_STRING_MSG.format(expected_string_regex.pattern, bag_command.output)
