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
import time
import shutil
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers

import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    ros2_topic1_pub_process = launch.actions.ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/rosbag2_record_topic1', 'test_msgs/msg/Strings', '{string_value: rosbag2 test message 1}'],
        output='screen', env=proc_env,
    )
    ros2_topic2_pub_process = launch.actions.ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/rosbag2_record_topic2', 'test_msgs/msg/Strings', '{string_value: rosbag2 test message 2}'],
        output='screen', env=proc_env,
    )

    return launch.LaunchDescription([
        ros2_topic1_pub_process,
        ros2_topic2_pub_process,
        launch_testing.actions.ReadyToTest()
    ]), locals()


class TestRecord(unittest.TestCase):

    def setUp(self):
        self.dir_path = 'rosbag2_tmp_bag'
        print('setting up record end to end test, deleting tmp bag file')
        try:
            shutil.rmtree(self.dir_path)
        except OSError as e:
            print("Error: %s : %s" % (self.dir_path, e.strerror))

    def tearDown(self):
        try:
            shutil.rmtree(self.dir_path)
        except OSError as e:
            print("Error: %s : %s" % (sel.dir_path, e.strerror))

    def test_ros2_bag_record(self, launch_service, proc_info, proc_output, proc_env):
        """Test terminating_proc without command line arguments."""
        print('Running ros2 bag record')

        rosbag_record_process = launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/rosbag2_record_topic1', '/rosbag2_record_topic2', '-o', self.dir_path],
            output='screen', env=proc_env,
        )
        with launch_testing.tools.launch_process(
              launch_service, rosbag_record_process, proc_info, proc_output) as command:
            command.wait_for_shutdown(timeout=10)
        proc_info.assertWaitForShutdown(rosbag_record_process, timeout=5)
        # not sure how to verify that process has stopped correctly.
        metadata_filepath = os.path.join(self.dir_path, 'metadata.yaml')
        assert(os.path.isfile(metadata_filepath))

        with open(metadata_filepath, 'r') as metadata_file:
            filetext = metadata_file.read()
            matches_messages = re.search(r'message_count:', filetext)
            matches_topic1 = re.search(r'/rosbag2_record_topic1', filetext)
            matches_topic2 = re.search(r'/rosbag2_record_topic2', filetext)
            assert(matches_messages != None)
            assert(matches_topic1 != None)
            assert(matches_topic2 != None)
