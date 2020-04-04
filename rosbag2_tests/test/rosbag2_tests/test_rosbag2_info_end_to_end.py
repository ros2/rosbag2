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

rosbag_info_process = launch.actions.ExecuteProcess(
    cmd=['ros2', 'bag', 'info', launch.substitutions.LaunchConfiguration('bag_file_path')],
    output='screen', env=proc_env,
)

rosbag_info_process_wrong = launch.actions.ExecuteProcess(
    cmd=['ros2', 'bag', 'info', '/path/to/non/existing/bag_file'],
    output='screen', env=proc_env,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'bag_file_path',
            description='Absolute path to a bag file.',
            default_value='../resources/cdr_test'
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestInfo(unittest.TestCase):

    def test_ros2_bag_info(self, launch_service, proc_info, proc_output):
        """Test terminating_proc without command line arguments."""
        print('Running ros2 bag info')
        with launch_testing.tools.launch_process(
              launch_service, rosbag_info_process, proc_info, proc_output) as command:
            assert command.wait_for_shutdown(timeout=3)
            assert command.exit_code == launch_testing.asserts.EXIT_OK
        # Using the SequentialStdout context manager asserts that the following stdout
        # happened in the same order that it's checked
        with launch_testing.asserts.assertSequentialStdout(proc_output, rosbag_info_process) as cm:
            cm.assertInStdout('Files:             cdr_test.db3')
            # cm.assertInStdout('Bag size:          .*B')
            cm.assertInStdout('Storage id:        sqlite3')
            cm.assertInStdout('Duration:          0.155s')
            # cm.assertInStdout('Start:             Sep 18 2018 .*:.*:44.241 (1537282604.241)')
            # cm.assertInStdout('End                Sep 18 2018 .*:.*:44.397 (1537282604.397)')
            cm.assertInStdout('Messages:          7')
            cm.assertInStdout(
                ('Topic information: Topic: /test_topic | Type: test_msgs/BasicTypes '
                 '| Count: 3 | Serialization Format: cdr'))
            cm.assertInStdout(
                ('Topic: /array_topic | Type: test_msgs/Arrays '
                 '| Count: 4 | Serialization Format: cdr'))
        # TODO(Karsten1987): Regex doesn't seem to work.
        # launch_testing.asserts.assertInStdout(
        #     proc_output, 'Files:\s+cdr_test.db3', rosbag_info_process)

    def test_ros2_bag_info_fail(self, launch_service, proc_info, proc_output):
        """Test ros2 bag info with a non existing bag file."""
        print('Running ros2 bag info with non existing bag file')
        with launch_testing.tools.launch_process(
              launch_service, rosbag_info_process_wrong, proc_info, proc_output) as command:
            assert command.wait_for_shutdown(timeout=3)
            assert command.exit_code == 1
        launch_testing.asserts.assertInStderr(
              proc_output, 'does not exist', rosbag_info_process_wrong)
