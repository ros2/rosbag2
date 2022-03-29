# Copyright 2020 Open Source Robotics Foundation, Inc.
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
from pathlib import Path
import shutil
import tempfile

import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_testing.asserts import EXIT_OK

import pytest


@pytest.mark.launch_test
def generate_test_description():
    tmp_dir_name = tempfile.mkdtemp()
    output_path = Path(tmp_dir_name) / 'ros2bag_test_record'
    record_all_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '--output', output_path.as_posix()],
        name='ros2bag-cli',
        output='screen',
    )

    return LaunchDescription([
        record_all_process,
        launch_testing.actions.ReadyToTest()
    ]), locals()


class TestRecord(unittest.TestCase):

    def test_output(self, record_all_process, proc_output):
        proc_output.assertWaitFor(
            'Listening for topics...',
            process=record_all_process
        )
        proc_output.assertWaitFor(
            "Subscribed to topic '/rosout'",
            process=record_all_process
        )


@launch_testing.post_shutdown_test()
class TestRecordAfterShutdown(unittest.TestCase):

    def test_exit_code(self, tmp_dir_name, record_all_process, proc_info):
        # Cleanup
        shutil.rmtree(tmp_dir_name, ignore_errors=True)

        # Check that the process exited with code 0
        launch_testing.asserts.assertExitCodes(
            proc_info,
            # SIGINT (2) is the typical exit code we see coming from rclcpp
            # On Windows, we get value '1'
            allowable_exit_codes=[EXIT_OK, 2] if os.name != 'nt' else [EXIT_OK, 1, 2],
            process=record_all_process
        )
