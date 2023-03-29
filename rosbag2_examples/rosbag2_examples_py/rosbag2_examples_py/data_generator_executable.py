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
from example_interfaces.msg import Int32
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.serialization import serialize_message
import rosbag2_py


def main(args=None):
    writer = rosbag2_py.SequentialWriter()

    storage_options = rosbag2_py._storage.StorageOptions(
        uri='big_synthetic_bag',
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    topic_info = rosbag2_py._storage.TopicMetadata(
        name='synthetic',
        type='example_interfaces/msg/Int32',
        serialization_format='cdr')
    writer.create_topic(topic_info)

    time_stamp = Clock().now()
    for ii in range(0, 100):
        data = Int32()
        data.data = ii
        writer.write(
            'synthetic',
            serialize_message(data),
            time_stamp.nanoseconds)
        time_stamp += Duration(seconds=1)


if __name__ == '__main__':
    main()
