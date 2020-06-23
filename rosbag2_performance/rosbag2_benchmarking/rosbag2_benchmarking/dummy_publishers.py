# Copyright 2020, Robotec.ai sp. z o.o.
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
"""Dummy publisher for warming up rosbag2 topics."""

import time

import rclpy
from rclpy.node import Node


class DummyPublisherUtility(Node):
    """Dummy publisher creates publishers for warming up topics on rosbag2."""

    __publishers = []

    def __init__(self):
        """Initialize class with required topics and corresponding types."""
        super().__init__('dummy_publisher')
        self.declare_parameter('topics')
        self.declare_parameter('types')

        topics = self.get_parameter(
            'topics'
        ).get_parameter_value().string_array_value
        types = self.get_parameter(
            'types'
        ).get_parameter_value().string_array_value

        if len(topics) != len(types):
            raise RuntimeError('Topics and types length mismatch.')

        for i in range(0, len(topics)):
            self.__warm_up_topic(topics[i], types[i])
            time.sleep(0.01)

    def __warm_up_topic(self, topic, type_):
        if type_ == 'sensor_msgs/msg/Image':
            from sensor_msgs.msg import Image
            self.__publishers.append(
                self.create_publisher(Image, topic, 1)
            )
        elif type_ == 'sensor_msgs/msg/PointCloud2':
            from sensor_msgs.msg import PointCloud2
            self.__publishers.append(
                self.create_publisher(PointCloud2, topic, 1)
            )
        elif type_ == 'std_msgs/msg/ByteMultiArray':
            from std_msgs.msg import ByteMultiArray
            self.__publishers.append(
                self.create_publisher(ByteMultiArray, topic, 1)
            )
        else:
            # (piotr.jaroszek) TODO: fill out rest or make a dynamic import
            pass


def main():
    """Ros2 once-spin run."""
    rclpy.init()
    node = DummyPublisherUtility()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        rclpy.shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
