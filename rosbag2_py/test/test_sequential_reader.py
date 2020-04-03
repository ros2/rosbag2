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

import pytest

from rosidl_runtime_py.utilities import get_message

from rclpy.serialization import deserialize_message

from std_msgs.msg import String
from rcl_interfaces.msg import Log

import rosbag2_py._rosbag2_py as rosbag2_py

def test_sequential_reader():

  storage_options = rosbag2_py.StorageOptions()
  storage_options.uri = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'resources', 'talker')
  storage_options.storage_id = 'sqlite3'

  converter_options = rosbag2_py.ConverterOptions()
  converter_options.input_serialization_format = 'cdr'
  converter_options.output_serialization_format = 'cdr'

  reader = rosbag2_py.Reader('SequentialReader')
  reader.open(storage_options, converter_options)

  topic_types = reader.get_all_topics_and_types()

  # Create a map for quicker lookup
  type_map = {topic_types[i].name : topic_types[i].type for i in range(len(topic_types))}

  # TODO(mabelzhang) Depends on ros2/rosbag2#302
  # Set a filter for topic name
  # storage_filter = rosbag2_py.StorageFilter()
  # storage_filter.topics = ['/topic']
  # reader.set_filter(storage_filter)

  while reader.has_next():
    (topic, data, t) = reader.read_next()
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(data, msg_type)

    assert msg_type == type(Log()) or msg_type == type(String())

    if msg_type == type(String()):
      assert msg.data[:14] == "Hello, world! "

  # reader.reset_filter(storage_filter)
