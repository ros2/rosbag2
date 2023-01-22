# Copyright 2023 Open Source Robotics Foundation, Inc.
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

import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
import launch_testing.actions
import launch_testing.asserts

import pytest

from rosbag2_test_common import TESTED_STORAGE_IDS

UNIQUE_PRESET_PROFILES = {
    'mcap': ['zstd_small'],
    'sqlite3': ['resilient'],
}


@pytest.mark.launch_test
@launch_testing.parametrize('storage_id', TESTED_STORAGE_IDS)
@launch_testing.markers.keep_alive
def generate_test_description(storage_id):
    return LaunchDescription([
        launch_testing.actions.ReadyToTest()
    ]), {'storage_id': storage_id}


class TestCLIExtension(unittest.TestCase):

    def test_output(self, launch_service, proc_info, proc_output, storage_id):
        unique_profiles = UNIQUE_PRESET_PROFILES[storage_id]
        help_proc = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', storage_id, '--help'],
            name='ros2bag-cli',
            output='screen')
        with launch_testing.tools.launch_process(
            launch_service, help_proc, proc_info, proc_output
        ):
            proc_info.assertWaitForShutdown(process=help_proc, timeout=4)
            for prof in unique_profiles:
                launch_testing.asserts.assertInStdout(
                    proc_output, prof, help_proc)
        launch_testing.asserts.assertExitCodes(proc_info, process=help_proc)
