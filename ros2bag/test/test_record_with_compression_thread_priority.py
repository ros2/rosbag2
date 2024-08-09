# Copyright 2024 Open Source Robotics Foundation, Inc.
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

from pathlib import Path
import tempfile

import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
import launch_testing.actions

import pytest


@pytest.mark.launch_test
def generate_test_description():
    tmp_dir_name = tempfile.mkdtemp()
    output_path = Path(tmp_dir_name) / 'ros2bag_test_record_with_compression_thread_priority'
    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '--output', output_path.as_posix(),
             '--log-level', 'debug', '--compression-threads-priority', '1',
             '--compression-mode', 'file', '--compression-format', 'zstd'],
        name='ros2bag-cli',
        output='screen',
    )

    return LaunchDescription([
        record_process,
        launch_testing.actions.ReadyToTest()
    ]), locals()


class TestRecordWithCompressionThreadPriority(unittest.TestCase):

    def test_priority_propagated_into_compression_thread(
            self, record_process, proc_output):
        proc_output.assertWaitFor(
            'Setting compression thread priority to 1',
            timeout=45,
            process=record_process
        )
