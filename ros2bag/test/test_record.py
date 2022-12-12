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
from rosbag2_test_common import TESTED_STORAGE_IDS


@pytest.mark.launch_test
def generate_test_description():
    tmp_dir_name = tempfile.mkdtemp()
    output_paths = {}
    processes = []

    for storage_id in TESTED_STORAGE_IDS:
        output_path = Path(tmp_dir_name) / f'ros2bag_test_record_{storage_id}'
        recorder_process = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', '-s', storage_id, '-a', '--output', output_path.as_posix()
            ],
            name=f'ros2bag-cli-{storage_id}',
            output='screen',
        )
        processes.append(recorder_process)
        output_paths[storage_id] = output_path

    return LaunchDescription(processes + [launch_testing.actions.ReadyToTest()]), {
        'output_paths': output_paths,
        'recorder_processes': processes,
        'tmp_dir_name': tmp_dir_name,
    }


class TestRecord(unittest.TestCase):

    def test_output(self, recorder_processes, proc_output):
        for recorder_process in recorder_processes:
            proc_output.assertWaitFor(
                'Listening for topics...',
                process=recorder_process
            )
            proc_output.assertWaitFor(
                "Subscribed to topic '/rosout'",
                process=recorder_process
            )


@launch_testing.post_shutdown_test()
class TestRecordAfterShutdown(unittest.TestCase):

    def test_exit_code(self, tmp_dir_name, output_paths, recorder_processes, proc_info):
        for storage_id, output_path in output_paths.items():
            assert output_path.is_dir(), f'failed to create bag dir for storage ID {storage_id}'
            assert (output_path / 'metadata.yaml').is_file(), f'no metadata.yaml for {storage_id}'
        # Cleanup
        shutil.rmtree(tmp_dir_name, ignore_errors=True)

        # Check that the process exited with code 0
        for process in recorder_processes:
            launch_testing.asserts.assertExitCodes(
                proc_info,
                # SIGINT (2) is the typical exit code we see coming from rclcpp
                # On Windows, we get value '1'
                allowable_exit_codes=[EXIT_OK, 2] if os.name != 'nt' else [EXIT_OK, 1, 2],
                process=process
            )
