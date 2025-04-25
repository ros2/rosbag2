# Copyright 2025 Apex.AI, Inc.
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

"""Script that reads ROS 2 messages from a bag file and saves them to a csv file."""

import argparse
import csv
import os

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.convert import message_to_yaml
from rosidl_runtime_py.utilities import get_message


def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag),
        rosbag2_py.ConverterOptions('', ''),
    )

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '-i', '--input', help='input bag path (folder or filepath) to read from'
    )

    args = parser.parse_args()

    if 'BUILD_WORKING_DIRECTORY' in os.environ:
        # Workaround for Bazel to support the relative path's for input and output files
        os.chdir(os.environ['BUILD_WORKING_DIRECTORY'])

    with open(f'{args.input}.csv', 'w', newline='') as f_out:
        csv_writer = csv.writer(f_out)
        # Write header
        csv_writer.writerow(['topic_name', 'topic_type', 'timestamp_ns', 'data'])

        for topic_name, msg, timestamp_ns in read_messages(args.input):
            # Convert to YAML
            yaml_str = message_to_yaml(msg)
            # Write the YAML string in the 'data' field
            csv_writer.writerow([topic_name, type(msg).__name__, timestamp_ns, yaml_str])
            # print(f"{topic_name} ({type(msg).__name__}) [{timestamp_ns}]: '{yaml_str}'")


if __name__ == '__main__':
    main()
