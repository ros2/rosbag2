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
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions

import pytest


RESOURCES_PATH = Path(__file__).parent / 'resources'
EXPECTED_OUTPUT = """
Files:             empty_bag_0.db3
Bag size:          49.1 KiB
Storage id:        sqlite3
Duration:          0.0s
Start:             Apr 11 2262 23:47:16.854 (9223372036.854)
End:               Apr 11 2262 23:47:16.854 (9223372036.854)
Messages:          0
Topic information: Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent \
| Count: 0 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log \
| Count: 0 | Serialization Format: cdr

"""
EXPECTED_TOPIC_NAME_OUTPUT = """/parameter_events
/rosout
"""


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestRos2BagInfo(unittest.TestCase):

    @classmethod
    def setUpClass(cls, launch_service, proc_info, proc_output):
        @contextlib.contextmanager
        def launch_bag_command(self, arguments, **kwargs):
            pkg_command_action = ExecuteProcess(
                cmd=['ros2', 'bag', *arguments],
                additional_env={
                    'PYTHONUNBUFFERED': '1',
                    'TZ': 'UTC',
                },
                name='ros2bag-cli',
                output='screen',
                **kwargs
            )
            with launch_testing.tools.launch_process(
                    launch_service, pkg_command_action, proc_info, proc_output
            ) as pkg_command:
                yield pkg_command
        cls.launch_bag_command = launch_bag_command

    def test_info_with_no_options(self):
        """Test the output with no options."""
        bag_path = RESOURCES_PATH / 'empty_bag'
        arguments = ['info', bag_path.as_posix()]
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_shutdown(timeout=5)
        assert bag_command.output == EXPECTED_OUTPUT, \
            'ros2bag CLI did not produce the expected output'

    def test_info_with_topic_name_option(self):
        """Test the output with --topic-name options."""
        bag_path = RESOURCES_PATH / 'empty_bag'
        arguments = ['info', '--topic-name', bag_path.as_posix()]
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_shutdown(timeout=5)
        assert bag_command.output == EXPECTED_TOPIC_NAME_OUTPUT, \
            'ros2bag CLI did not produce the expected output'
