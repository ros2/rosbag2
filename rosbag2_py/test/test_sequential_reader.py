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
from pathlib import Path

from common import get_rosbag_options, TESTED_STORAGE_IDS

import pytest

from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


RESOURCES_PATH = Path(os.environ['ROSBAG2_PY_TEST_RESOURCES_DIR'])


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_sequential_reader(storage_id):
    bag_path = str(RESOURCES_PATH / storage_id / 'talker')
    storage_options, converter_options = get_rosbag_options(bag_path, storage_id)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # Set filter for topic of string type
    storage_filter = rosbag2_py.StorageFilter(topics=['/topic'])
    reader.set_filter(storage_filter)

    msg_counter = 0

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        assert isinstance(msg, String)
        assert msg.data == f'Hello, world! {msg_counter}'

        msg_counter += 1

    # No filter
    reader.reset_filter()

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    msg_counter = 0

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        assert isinstance(msg, Log) or isinstance(msg, String)

        if isinstance(msg, String):
            assert msg.data == f'Hello, world! {msg_counter}'
            msg_counter += 1


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_sequential_reader_seek(storage_id):
    bag_path = str(RESOURCES_PATH / storage_id / 'talker')
    storage_options, converter_options = get_rosbag_options(bag_path, storage_id)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # Seek No Filter
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    reader.seek(1585866237113147888)

    msg_counter = 5

    (topic, data, t) = reader.read_next()
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(data, msg_type)

    assert isinstance(msg, Log)

    (topic, data, t) = reader.read_next()
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(data, msg_type)

    isinstance(msg, String)
    assert msg.data == f'Hello, world! {msg_counter}'
    msg_counter += 1

    # Set Filter will continue
    storage_filter = rosbag2_py.StorageFilter(topics=['/topic'])
    reader.set_filter(storage_filter)

    (topic, data, t) = reader.read_next()
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(data, msg_type)
    isinstance(msg, String)
    assert msg.data == f'Hello, world! {msg_counter}'

    # Seek will keep filter
    reader.seek(1585866239113147888)

    msg_counter = 8

    (topic, data, t) = reader.read_next()
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(data, msg_type)
    isinstance(msg, String)
    assert msg.data == f'Hello, world! {msg_counter}'
    msg_counter += 1

    (topic, data, t) = reader.read_next()
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(data, msg_type)
    isinstance(msg, String)
    assert msg.data == f'Hello, world! {msg_counter}'


def test_plugin_list():
    reader_plugins = rosbag2_py.get_registered_readers()
    assert 'my_read_only_test_plugin' in reader_plugins
