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
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension
from ros2cli.node import NODE_NAME_PREFIX
from rosbag2_py import get_registered_writers
from rosbag2_py import Recorder
from rosbag2_py import RecordOptions
from rosbag2_py import StorageOptions
import yaml


class RecordVerb(VerbExtension):
    """Record ROS data to a bag."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        writer_choices = get_registered_writers()
        default_writer = 'sqlite3' if 'sqlite3' in writer_choices else writer_choices[0]

        parser.add_argument(
            '-a', '--all', action='store_true',
            help='recording all topics, required if no topics '
            'are listed explicitly or through a regex')
        parser.add_argument(
            'topics', nargs='*', default=None, help='topics to be recorded')
        parser.add_argument(
            '-e', '--regex', default='', help='recording only topics '
            'matching provided regular expression')
        parser.add_argument(
            '-x', '--exclude', default='', help='exclude topics '
            'matching provided regular expression. Works with -a and -e, '
            'subtracting excluded topics')
        parser.add_argument(
            '-o', '--output',
            help='destination of the bagfile to create, \
            defaults to a timestamped folder in the current directory')
        parser.add_argument(
            '-s', '--storage', default=default_writer, choices=writer_choices,
            help=f"storage identifier to be used, defaults to '{default_writer}'")
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
            '-d', '--max-bag-duration', type=int, default=0,
            help='maximum duration in seconds before the bagfile will be split. '
                  'Default is zero, recording written in single bagfile and splitting '
                  'is disabled. If both splitting by size and duration are enabled, '
                  'the bag will split at whichever threshold is reached first.'
        )
        parser.add_argument(
            '--max-cache-size', type=int, default=100*1024*1024,
            help='maximum size (in bytes) of messages to hold in each buffer of cache.'
                 'Default is 100 mebibytes. The cache is handled through double buffering, '
                 'which means that in pessimistic case up to twice the parameter value of memory'
                 'is needed. A rule of thumb is to cache an order of magitude corresponding to'
                 'about one second of total recorded data volume.'
                 'If the value specified is 0, then every message is directly written to disk.'
        )
        parser.add_argument(
            '--compression-mode', type=str, default='none',
            choices=['none', 'file', 'message'],
            help="Determine whether to compress by file or message. Default is 'none'."
        )
        parser.add_argument(
            '--compression-format', type=str, default='', choices=['zstd'],
            help='Specify the compression format/algorithm. Default is none.'
        )
        parser.add_argument(
            '--compression-queue-size', type=int, default=1,
            help='Number of files or messages that may be queued for compression '
                 'before being dropped.  Default is 1.'
        )
        parser.add_argument(
            '--compression-threads', type=int, default=0,
            help='Number of files or messages that may be compressed in parallel. '
                 'Default is 0, which will be interpreted as the number of CPU cores.'
        )
        parser.add_argument(
            '--include-hidden-topics', action='store_true',
            help='record also hidden topics.'
        )
        parser.add_argument(
            '--qos-profile-overrides-path', type=FileType('r'),
            help='Path to a yaml file defining overrides of the QoS profile for specific topics.'
        )
        parser.add_argument(
            '--storage-preset-profile', type=str, default='none', choices=['none', 'resilient'],
            help='Select a configuration preset for storage.'
                 'resilient (sqlite3):'
                 'indicate preference for avoiding data corruption in case of crashes,'
                 'at the cost of performance. Setting this flag disables optimization settings '
                 'for storage (the defaut). This flag settings can still be overriden by '
                 'corresponding settings in the config passed with --storage-config-file.'
        )
        parser.add_argument(
            '--storage-config-file', type=FileType('r'),
            help='Path to a yaml file defining storage specific configurations. '
                 'For the default storage plugin settings are specified through syntax:'
                 'write:'
                 '  pragmas: [\"<setting_name>\" = <setting_value>]'
                 'For a list of sqlite3 settings, refer to sqlite3 documentation')
        self._subparser = parser

    def main(self, *, args):  # noqa: D102
        # both all and topics cannot be true
        if (args.all and (args.topics or args.regex)) or (args.topics and args.regex):
            return print_error('Must specify only one option out of topics, --regex or --all')
        # one out of "all", "topics" and "regex" must be true
        if not(args.all or (args.topics and len(args.topics) > 0) or (args.regex)):
            return print_error('Invalid choice: Must specify topic(s), --regex or --all')

        if args.topics and args.exclude:
            return print_error('--exclude argument cannot be used when specifying a list '
                               'of topics explicitly')

        if args.exclude and not(args.regex or args.all):
            return print_error('--exclude argument requires either --all or --regex')

        uri = args.output or datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')

        if os.path.isdir(uri):
            return print_error("Output folder '{}' already exists.".format(uri))

        if args.compression_format and args.compression_mode == 'none':
            return print_error('Invalid choice: Cannot specify compression format '
                               'without a compression mode.')

        if args.compression_queue_size < 1:
            return print_error('Compression queue size must be at least 1.')

        args.compression_mode = args.compression_mode.upper()

        qos_profile_overrides = {}  # Specify a valid default
        if args.qos_profile_overrides_path:
            qos_profile_dict = yaml.safe_load(args.qos_profile_overrides_path)
            try:
                qos_profile_overrides = convert_yaml_to_qos_profile(
                    qos_profile_dict)
            except (InvalidQoSProfileException, ValueError) as e:
                return print_error(str(e))

        storage_config_file = ''
        if args.storage_config_file:
            storage_config_file = args.storage_config_file.name

        storage_options = StorageOptions(
            uri=uri,
            storage_id=args.storage,
            max_bagfile_size=args.max_bag_size,
            max_bagfile_duration=args.max_bag_duration,
            max_cache_size=args.max_cache_size,
            storage_preset_profile=args.storage_preset_profile,
            storage_config_uri=storage_config_file,
        )
        record_options = RecordOptions(
            all=args.all,
            regex=args.regex,
            exclude=args.exclude,
            is_discovery_disabled=args.no_discovery,
            topic_polling_interval=args.polling_interval,
            node_prefix=NODE_NAME_PREFIX,
            compression_mode=args.compression_mode,
            compression_format=args.compression_format,
            compression_queue_size=args.compression_queue_size,
            compression_threads=args.compression_threads,
            include_hidden_topics=args.include_hidden_topics,
            topic_qos_profile_overrides=qos_profile_overrides,
            topics=args.topics,
            rmw_serialization_format=args.serialization_format,
        )

        recorder = Recorder()
        recorder.record(storage_options, record_options)

        if os.path.isdir(uri) and not os.listdir(uri):
            os.rmdir(uri)
