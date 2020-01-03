# Copyright 2019-2020 Martin Idel
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

"""Launch a set of nodes publishing large messages"""
from sys import executable
from os.path import join

from launch import LaunchDescription
import launch.actions
import launch_ros.actions
import launch.event_handlers

from ament_index_python import get_package_prefix


def add_test_to_description(ls: LaunchDescription) -> LaunchDescription:
    package_string = get_package_prefix('rosbag2_performance_testing')
    perfomance_script = join(package_string, 'share', 'rosbag2_performance_testing', 'performance_test.py')
    test_script_action = launch.actions.ExecuteProcess(
        cmd=[executable, '-u', perfomance_script], name='rosbag_perfomance_test'
    )
    ls.add_action(test_script_action)

    def finished_test_event_handler(event):
        print(event.text.decode())
        if 'Performance test' in event.text.decode():
            return launch.actions.EmitEvent(event=launch.events.Shutdown(
                reason="Test finished"
            ))

    ls.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
        target_action=test_script_action,
        on_stdout=finished_test_event_handler,
        on_stderr=finished_test_event_handler,
    )))

    return ls


def generate_launch_description():
    ls = LaunchDescription([
        launch_ros.actions.Node(
            package='rosbag2_performance_testing', node_executable='image_publisher1', output='screen'),
        launch_ros.actions.Node(
            package='rosbag2_performance_testing', node_executable='image_publisher2', output='screen'),
        launch_ros.actions.Node(
            package='rosbag2_performance_testing', node_executable='pointcloud_publisher', output='screen')
    ])

    return add_test_to_description(ls)
