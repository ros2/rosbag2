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

import glob
import os
import time
import shutil

from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py._rosbag2_py as rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

from common import get_rosbag_options


def create_topic(writer, topic_name, topic_type, serialization_format='cdr'):
  """
  Create a new topic.

  :param writer: writer instance
  :param topic_name:
  :param topic_type:
  :param serialization_format:
  :return:
  """
  topic_name = topic_name
  topic = rosbag2_py.TopicMetadata()
  topic.name = topic_name
  topic.serialization_format = serialization_format
  topic.type = topic_type

  writer.create_topic(topic)


def test_sequential_writer():
  """
  Test for sequential writer.
  :return:
  """

  bag_path = os.path.join(
      os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'resources', 'tmp_write_test')

  if os.path.exists(bag_path):
    print('%s exists. It will be overwritten by new rosbag2_py test_sequential_writer run.' % (
      bag_path))
    shutil.rmtree(bag_path)
  os.mkdir(bag_path)

  storage_options, converter_options = get_rosbag_options(bag_path)

  writer = rosbag2_py.Writer('SequentialWriter')
  writer.open(storage_options, converter_options)

  # create topic
  topic_name = '/chatter'
  create_topic(writer, topic_name, 'std_msgs/msg/String')

  for i in range(10):
    msg = String()
    msg.data = f'Hello, world! {str(i)}'
    time_stamp = i * 100

    writer.write(topic_name, serialize_message(msg), time_stamp)

  # close bag and create new storage instance
  del writer
  storage_options, converter_options = get_rosbag_options(bag_path)

  reader = rosbag2_py.Reader('SequentialReader')
  reader.open(storage_options, converter_options)

  topic_types = reader.get_all_topics_and_types()

  # Create a map for quicker lookup
  type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

  msg_counter = 0
  while reader.has_next():
    topic, data, t = reader.read_next()
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(data, msg_type)

    assert msg_type == type(String())
    assert msg.data == 'Hello, world! %d' % msg_counter
    assert t == msg_counter * 100

    msg_counter += 1

  # Cleanup
  shutil.rmtree(bag_path)
