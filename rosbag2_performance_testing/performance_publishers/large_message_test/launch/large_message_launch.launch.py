# Copyright 2019 Martin Idel
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
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='rosbag2_performance_publishers', node_executable='image_publisher1', output='screen'),
        launch_ros.actions.Node(
            package='rosbag2_performance_publishers', node_executable='image_publisher2', output='screen'),
        launch_ros.actions.Node(
            package='rosbag2_performance_publishers', node_executable='pointcloud_publisher', output='screen')
    ])