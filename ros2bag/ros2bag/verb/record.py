# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from argparse import FileType
import datetime
import os

from rclpy.qos import InvalidQoSProfileException
from ros2bag.api import convert_yaml_to_qos_profile
from ros2bag.api import create_bag_directory
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension
from ros2cli.node import NODE_NAME_PREFIX
import yaml


class RecordVerb(VerbExtension):
    """Record ROS data to a bag."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            '-a', '--all', action='store_true',
            help='recording all topics, required if no topics are listed explicitly.')
        parser.add_argument(
            'topics', nargs='*', help='topics to be recorded')
        parser.add_argument(
            '-o', '--output',
            help='destination of the bagfile to create, \
            defaults to a timestamped folder in the current directory')
        parser.add_argument(
            '-s', '--storage', default='sqlite3',
            help="storage identifier to be used, defaults to 'sqlite3'")
        parser.add_argument(
            '-f', '--serialization-format', default='',
            help='rmw serialization format in which the messages are saved, defaults to the'
                 ' rmw currently in use')
        parser.add_argument(
            '--no-discovery', action='store_true',
            help='disables topic auto discovery during recording: only topics present at '
                 'startup will be recorded')
        parser.add_argument(
            '-p', '--polling-interval', type=int, default=100,
            help='time in ms to wait between querying available topics for recording. '
                  'It has no effect if --no-discovery is enabled.'
        )
        parser.add_argument(
            '-b', '--max-bag-size', type=int, default=0,
            help='maximum size in bytes before the bagfile will be split. '
                  'Default it is zero, recording written in single bagfile and splitting '
                  'is disabled.'
        )
        parser.add_argument(
            '--max-cache-size', type=int, default=0,
            help='maximum amount of messages to hold in cache before writing to disk. '
                 'Default it is zero, writing every message directly to disk.'
        )
        parser.add_argument(
            '--compression-mode', type=str, default='none',
            choices=['none', 'file'],
            help='Determine whether to compress bag files. Default is "none".'
        )
        parser.add_argument(
            '--compression-format', type=str, default='', choices=['zstd'],
            help='Specify the compression format/algorithm. Default is none.'
        )
        parser.add_argument(
            '--include-hidden-topics', action='store_true',
            help='record also hidden topics.'
        )
        parser.add_argument(
            '--qos-profile-overrides-path', type=FileType('r'),
            help='Path to a yaml file defining overrides of the QoS profile for specific topics.'
        )
        self._subparser = parser

    def main(self, *, args):  # noqa: D102
        if args.all and args.topics:
            return print_error('Invalid choice: Can not specify topics and -a at the same time.')

        uri = args.output or datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')

        if os.path.isdir(uri):
            return print_error("Output folder '{}' already exists.".format(uri))

        if args.compression_format and args.compression_mode == 'none':
            return print_error('Invalid choice: Cannot specify compression format '
                               'without a compression mode.')

        args.compression_mode = args.compression_mode.upper()

        qos_profile_overrides = {}  # Specify a valid default
        if args.qos_profile_overrides_path:
            qos_profile_dict = yaml.safe_load(args.qos_profile_overrides_path)
            try:
                qos_profile_overrides = convert_yaml_to_qos_profile(
                    qos_profile_dict)
            except (InvalidQoSProfileException, ValueError) as e:
                return print_error(str(e))

        create_bag_directory(uri)

        if args.all:
            # NOTE(hidmic): in merged install workspaces on Windows, Python entrypoint lookups
            #               combined with constrained environments (as imposed by colcon test)
            #               may result in DLL loading failures when attempting to import a C
            #               extension. Therefore, do not import rosbag2_transport at the module
            #               level but on demand, right before first use.
            from rosbag2_transport import rosbag2_transport_py

            rosbag2_transport_py.record(
                uri=uri,
                storage_id=args.storage,
                serialization_format=args.serialization_format,
                node_prefix=NODE_NAME_PREFIX,
                compression_mode=args.compression_mode,
                compression_format=args.compression_format,
                all=True,
                no_discovery=args.no_discovery,
                polling_interval=args.polling_interval,
                max_bagfile_size=args.max_bag_size,
                max_cache_size=args.max_cache_size,
                include_hidden_topics=args.include_hidden_topics,
                qos_profile_overrides=qos_profile_overrides)
        elif args.topics and len(args.topics) > 0:
            # NOTE(hidmic): in merged install workspaces on Windows, Python entrypoint lookups
            #               combined with constrained environments (as imposed by colcon test)
            #               may result in DLL loading failures when attempting to import a C
            #               extension. Therefore, do not import rosbag2_transport at the module
            #               level but on demand, right before first use.
            from rosbag2_transport import rosbag2_transport_py

            rosbag2_transport_py.record(
                uri=uri,
                storage_id=args.storage,
                serialization_format=args.serialization_format,
                node_prefix=NODE_NAME_PREFIX,
                compression_mode=args.compression_mode,
                compression_format=args.compression_format,
                no_discovery=args.no_discovery,
                polling_interval=args.polling_interval,
                max_bagfile_size=args.max_bag_size,
                max_cache_size=args.max_cache_size,
                topics=args.topics,
                include_hidden_topics=args.include_hidden_topics,
                qos_profile_overrides=qos_profile_overrides)
        else:
            self._subparser.print_help()

        if os.path.isdir(uri) and not os.listdir(uri):
            os.rmdir(uri)
