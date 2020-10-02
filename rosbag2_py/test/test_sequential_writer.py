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
import sys

from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

if os.environ.get('ROSBAG2_PY_TEST_WITH_RTLD_GLOBAL', None) is not None:
    # This is needed on Linux when compiling with clang/libc++.
    # TL;DR This makes class_loader work when using a python extension compiled with libc++.
    #
    # For the fun RTTI ABI details, see https://whatofhow.wordpress.com/2015/03/17/odr-rtti-dso/.
    sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)

from common import get_rosbag_options  # noqa
import rosbag2_py  # noqa


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
    topic = rosbag2_py.TopicMetadata(name=topic_name, type=topic_type,
                                     serialization_format=serialization_format)

    writer.create_topic(topic)


def test_sequential_writer(tmp_path):
    """
    Test for sequential writer.

    :return:
    """
    bag_path = str(tmp_path / 'tmp_write_test')

    storage_options, converter_options = get_rosbag_options(bag_path)

    writer = rosbag2_py.SequentialWriter()
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

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    msg_counter = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg_deserialized = deserialize_message(data, msg_type)

        assert isinstance(msg_deserialized, String)
        assert msg_deserialized.data == f'Hello, world! {msg_counter}'
        assert t == msg_counter * 100

        msg_counter += 1
