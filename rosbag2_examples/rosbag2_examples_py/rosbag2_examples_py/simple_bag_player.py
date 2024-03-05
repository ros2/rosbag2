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
import sys
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import rosbag2_py
from std_msgs.msg import String


class SimpleBagPlayer(Node):

    def __init__(self, bag_filename):
        super().__init__('simple_bag_player')
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='my_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)

        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        while self.reader.has_next():
            msg = self.reader.read_next()
            if msg[0] != 'chatter':
                continue
            ros_msg = deserialize_message(msg[1], String)
            self.publisher.publish(ros_msg)
            self.get_logger().info(ros_msg.data)



def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagPlayer(sys.argv[1])
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
