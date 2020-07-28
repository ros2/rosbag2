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

from pathlib import Path

from common import get_rosbag_options
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


def test_sequential_reader():
    bag_path = str(Path(__file__).parent.parent / 'resources' / 'talker')
    print('TEMP DEBUG bag_path: %s' % bag_path)
    storage_options, converter_options = get_rosbag_options(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # Set filter for topic of string type
    storage_filter = rosbag2_py.StorageFilter()
    storage_filter.topics = ['/topic']
    reader.set_filter(storage_filter)

    msg_counter = 0

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        assert isinstance(msg, String)
        assert msg.data == 'Hello, world! %d' % msg_counter

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
            assert msg.data == 'Hello, world! %d' % msg_counter
            msg_counter += 1
