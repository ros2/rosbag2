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
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from example_interfaces.msg import Int32

import rosbag2_py


class DataGeneratorNode(Node):
    def __init__(self):
        super().__init__('data_generator_node')
        self.data = Int32()
        self.data.data = 0
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri='timed_synthetic_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name='synthetic',
            type='example_interfaces/msg/Int32',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.writer.write(
            'synthetic',
            serialize_message(self.data),
            self.get_clock().now().nanoseconds)
        self.data.data += 1


def main(args=None):
    rclpy.init(args=args)
    dgn = DataGeneratorNode()
    rclpy.spin(dgn)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
