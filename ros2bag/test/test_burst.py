# Copyright 2022 Open Source Robotics Foundation, Inc.
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
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

import pytest


RESOURCES_PATH = Path(__file__).parent / 'resources'
TEST_NODE = 'ros2bag_burst_test_node'
TEST_NAMESPACE = 'ros2bag_record_qos_profile'
ERROR_STRING = r'\[ERROR] \[ros2bag]:'


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestRos2BagBurst(unittest.TestCase):
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

    def test_burst(self):
        """Test the burst mode of playback."""
        bag_path = RESOURCES_PATH / 'empty_bag'
        arguments = ['burst', bag_path.as_posix()]
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_shutdown(timeout=5)
        expected_string_regex = re.compile(ERROR_STRING)
        matches = expected_string_regex.search(bag_command.output)
        assert not matches, 'ros2bag CLI did not produce the expected output'
