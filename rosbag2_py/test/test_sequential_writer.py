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
import glob
import pytest
import time

from rosidl_runtime_py.utilities import get_message

from rclpy.serialization import serialize_message

from std_msgs.msg import String

import rosbag2_py._rosbag2_py as rosbag2_py


def test_sequential_writer():
    bag_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'resources', 'wtalker')

    for f in glob.glob(f"{bag_path}/*"):
        os.remove(f)

    storage_options = rosbag2_py.StorageOptions()
    storage_options.uri = bag_path
    storage_options.storage_id = 'sqlite3'

    serialization_format = 'cdr'
    converter_options = rosbag2_py.ConverterOptions()
    converter_options.input_serialization_format = serialization_format
    converter_options.output_serialization_format = serialization_format

    writer = rosbag2_py.Writer('SequentialWriter')
    writer.open(storage_options, converter_options)

    # create topic
    topic = rosbag2_py.TopicMetadata()
    topic.name = "chatter"
    topic.serialization_format = serialization_format
    topic.type = "std_msgs/String"

    writer.create_topic(topic)

    for i in range(10):
        msg = String()
        msg.data = f"Hello {str(i)}"
        serialized_data = serialize_message(msg)
        bag_msg = rosbag2_py.SerializedBagMessage()
        bag_msg.topic_name = "chatter"
        bag_msg.serialized_data = serialized_data
        bag_msg.time_stamp = int(round(time.time() * 1000))
        writer.write(bag_msg)
