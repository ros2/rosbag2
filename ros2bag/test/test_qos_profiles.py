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
import tempfile

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
TEST_NODE = 'cli_qos_profile_test_node'
TEST_NAMESPACE = 'cli_qos_profile'
ERROR_STRING = 'ERROR:ros2bag:'


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestRos2BagCLI(unittest.TestCase):

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

    def test_qos_simple(self):
        profile_path = PROFILE_PATH / 'qos_profile.yaml'
        with tempfile.TemporaryDirectory() as tmpdirname:
            output_path = Path(tmpdirname) / 'ros2bag_test_basic'
            arguments = ['record', '-a', '--qos-profile-overrides', profile_path.as_posix(),
                         '--output', output_path.as_posix()]
            with self.launch_bag_command(arguments=arguments) as bag_command:
                assert bag_command.wait_for_output(timeout=5), \
                    print('ros2bag CLI failed to start.')
            output = bag_command.output.splitlines()
            for line in output:
                assert ERROR_STRING not in line, \
                    print('ros2bag CLI failed: {}'.format(line))

    def test_incomplete_qos_profile(self):
        profile_path = PROFILE_PATH / 'incomplete_qos_profile.yaml'
        with tempfile.TemporaryDirectory() as tmpdirname:
            output_path = Path(tmpdirname) / 'ros2bag_test_incomplete'
            arguments = ['record', '-a', '--qos-profile-overrides', profile_path.as_posix(),
                         '--output', output_path.as_posix()]
            with self.launch_bag_command(arguments=arguments) as bag_command:
                assert bag_command.wait_for_output(timeout=5), \
                    print('ros2bag CLI failed to start.')
            output = bag_command.output.splitlines()
            for line in output:
                assert ERROR_STRING not in line, \
                    print('ros2bag CLI failed: {}'.format(line))

    def test_nonexistent_qos_profile(self):
        profile_path = PROFILE_PATH / 'foobar.yaml'
        with tempfile.TemporaryDirectory() as tmpdirname:
            output_path = Path(tmpdirname) / 'ros2bag_test_incomplete'
            arguments = ['record', '-a', '--qos-profile-overrides', profile_path.as_posix(),
                         '--output', output_path.as_posix()]
            with self.launch_bag_command(arguments=arguments) as bag_command:
                assert bag_command.wait_for_output(timeout=5), \
                    print('ros2bag CLI failed to start.')
            output = bag_command.output.splitlines()
            for line in output:
                assert ERROR_STRING in line, \
                    print('ros2bag CLI succeeded when is should have failed: {}'.format(line))
