# Copyright 2020 Open Source Robotics Foundation, Inc
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

import os
import re
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers

import pytest


# This is necessary to get unbuffered output from the process under test
proc_env = os.environ.copy()
proc_env['PYTHONUNBUFFERED'] = '1'

rosbag_play_process = launch.actions.ExecuteProcess(
    cmd=['ros2', 'bag', 'play', launch.substitutions.LaunchConfiguration('bag_file_path')],
    output='screen', env=proc_env,
)

ros2_topic_echo_process = launch.actions.ExecuteProcess(
    cmd=['ros2', 'topic', 'echo', '/test_msg_strings', 'test_msgs/msg/Strings'],
    output='screen', env=proc_env,
    sigterm_timeout='20',
    sigkill_timeout='10'
)


@pytest.mark.launch_test
#@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'bag_file_path',
            description='Absolute path to a bag file.',
            default_value='../resources/cdr_test'
        ),

        ros2_topic_echo_process,

        rosbag_play_process,

        launch_testing.actions.ReadyToTest(),
    ])


class TestPlay(unittest.TestCase):

    def test_ros2_bag_play(self, launch_service, proc_info, proc_output):
        """Test that ros2 topic echo received messages."""
        print('Analyzing ros2 topic echo output')
        proc_info.assertWaitForShutdown(rosbag_play_process, timeout=15)
        p = re.compile(
            (r"(string_value: rosbag2 test message\n"
             r"string_value_default1: Hello world!\n"
             r"string_value_default2: Hello'world!\n"
             r"string_value_default3: Hello\"world!\n"
             r"string_value_default4: Hello\'world!\n"
             r"string_value_default5: Hello\"world!\n"
             r"bounded_string_value: ''\n"
             r"bounded_string_value_default1: Hello world!\n"
             r"bounded_string_value_default2: Hello\'world!\n"
             r"bounded_string_value_default3: Hello\"world!\n"
             r"bounded_string_value_default4: Hello\'world!\n"
             r"bounded_string_value_default5: Hello\"world!\n"
             r"---\n){8}"))
        launch_testing.asserts.assertInStdout(proc_output, p, ros2_topic_echo_process)
