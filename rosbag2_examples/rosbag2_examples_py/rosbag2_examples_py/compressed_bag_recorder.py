# Copyright 2025 Open Source Robotics Foundation, Inc.
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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py
from std_msgs.msg import String


class CompressedBagRecorder(Node):

    def __init__(self):
        super().__init__('compressed_bag_recorder')

        compression_options = rosbag2_py.CompressionOptions(
            compression_format='zstd',
            compression_mode=rosbag2_py.CompressionMode.MESSAGE)

        storage_options = rosbag2_py.StorageOptions(
            uri='my_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')

        self.compressed_writer = rosbag2_py.SequentialCompressionWriter(compression_options)
        self.compressed_writer.open(storage_options, converter_options)

        topic_info = rosbag2_py.TopicMetadata(
            id=0,
            name='chatter',
            type='std_msgs/msg/String',
            serialization_format='cdr')
        self.compressed_writer.create_topic(topic_info)

        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.topic_callback,
            10)
        self.subscription

    def topic_callback(self, msg):
        self.compressed_writer.write(
            'chatter',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def main(args=None):
    try:
        with rclpy.init(args=args):
            cbr = CompressedBagRecorder()
            rclpy.spin(cbr)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
