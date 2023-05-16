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
from ros2bag.api import add_writer_storage_plugin_extensions
from ros2bag.api import convert_yaml_to_qos_profile
from ros2bag.api import print_error
from ros2bag.api import SplitLineFormatter
from ros2bag.verb import VerbExtension
from ros2cli.node import NODE_NAME_PREFIX
from rosbag2_py import get_default_storage_id
from rosbag2_py import get_registered_compressors
from rosbag2_py import get_registered_serializers
from rosbag2_py import get_registered_writers
from rosbag2_py import Recorder
from rosbag2_py import RecordOptions
from rosbag2_py import StorageOptions
import yaml


class RecordVerb(VerbExtension):
    """Record ROS data to a bag."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.formatter_class = SplitLineFormatter
        writer_choices = get_registered_writers()
        default_storage_id = get_default_storage_id()
        default_writer = default_storage_id if default_storage_id in writer_choices else \
            next(iter(writer_choices))

        serialization_choices = get_registered_serializers()
        converter_suffix = '_converter'
        serialization_choices = {
            f[:-len(converter_suffix)]
            for f in serialization_choices
            if f.endswith(converter_suffix)
        }

        # Base output
        parser.add_argument(
            '-o', '--output',
            help='Destination of the bagfile to create, '
                 'defaults to a timestamped folder in the current directory.')
        parser.add_argument(
            '-s', '--storage', default=default_writer, choices=writer_choices,
            help="Storage identifier to be used, defaults to '%(default)s'.")

        # Topic filter arguments
        parser.add_argument(
            'topics', nargs='*', default=None, help='List of topics to record.')
        parser.add_argument(
            '-a', '--all', action='store_true',
            help='Record all topics. Required if no explicit topic list or regex filters.')
        parser.add_argument(
            '-e', '--regex', default='',
            help='Record only topics containing provided regular expression. '
                 'Overrides --all, applies on top of topics list.')
        parser.add_argument(
            '-x', '--exclude', default='',
            help='Exclude topics containing provided regular expression. '
                 'Works on top of --all, --regex, or topics list.')

        # Discovery behavior
        parser.add_argument(
            '--include-unpublished-topics', action='store_true',
            help='Discover and record topics which have no publisher. '
                 'Subscriptions on such topics will be made with default QoS unless otherwise '
                 'specified in a QoS overrides file.')
        parser.add_argument(
            '--include-hidden-topics', action='store_true',
            help='Discover and record hidden topics as well. '
                 'These are topics used internally by ROS 2 implementation.')
        parser.add_argument(
            '--no-discovery', action='store_true',
            help='Disables topic auto discovery during recording: only topics present at '
                 'startup will be recorded.')
        parser.add_argument(
            '-p', '--polling-interval', type=int, default=100,
            help='Time in ms to wait between querying available topics for recording. '
                 'It has no effect if --no-discovery is enabled.')
        parser.add_argument(
            '--ignore-leaf-topics', action='store_true',
            help='Ignore topics without a publisher.')
        parser.add_argument(
            '--qos-profile-overrides-path', type=FileType('r'),
            help='Path to a yaml file defining overrides of the QoS profile for specific topics.')

        # Core config
        parser.add_argument(
            '-f', '--serialization-format', default='', choices=serialization_choices,
            help='The rmw serialization format in which the messages are saved, defaults to the '
                 'rmw currently in use.')
        parser.add_argument(
            '-b', '--max-bag-size', type=int, default=0,
            help='Maximum size in bytes before the bagfile will be split. '
                 'Default: %(default)d, recording written in single bagfile and splitting '
                 'is disabled.')
        parser.add_argument(
            '-d', '--max-bag-duration', type=int, default=0,
            help='Maximum duration in seconds before the bagfile will be split. '
                 'Default: %(default)d, recording written in single bagfile and splitting '
                 'is disabled. If both splitting by size and duration are enabled, '
                 'the bag will split at whichever threshold is reached first.')
        parser.add_argument(
            '--max-cache-size', type=int, default=100*1024*1024,
            help='Maximum size (in bytes) of messages to hold in each buffer of cache. '
                 'Default: %(default)d. The cache is handled through double buffering, '
                 'which means that in pessimistic case up to twice the parameter value of memory '
                 'is needed. A rule of thumb is to cache an order of magnitude corresponding to '
                 'about one second of total recorded data volume. '
                 'If the value specified is 0, then every message is directly written to disk.')
        parser.add_argument(
            '--start-paused', action='store_true', default=False,
            help='Start the recorder in a paused state.')
        parser.add_argument(
            '--use-sim-time', action='store_true', default=False,
            help='Use simulation time.')
        parser.add_argument(
            '--node-name', type=str, default='rosbag2_recorder',
            help='Specify the recorder node name. Default is %(default)s.')
        parser.add_argument(
            '--custom-data', type=str, metavar='KEY=VALUE', nargs='*',
            help='Store the custom data in metadata.yaml '
                 'under "rosbag2_bagfile_information/custom_data". The key=value pair can '
                 'appear more than once. The last value will override the former ones.')
        parser.add_argument(
            '--snapshot-mode', action='store_true',
            help='Enable snapshot mode. Messages will not be written to the bagfile until '
                 'the "/rosbag2_recorder/snapshot" service is called.')

        # Storage configuration
        add_writer_storage_plugin_extensions(parser)

        # Core compression configuration
        # TODO(emersonknapp) this configuration will be moved down to implementing plugins
        parser.add_argument(
            '--compression-queue-size', type=int, default=1,
            help='Number of files or messages that may be queued for compression '
                 'before being dropped.  Default is %(default)d.')
        parser.add_argument(
            '--compression-threads', type=int, default=0,
            help='Number of files or messages that may be compressed in parallel. '
                 'Default is %(default)d, which will be interpreted as the number of CPU cores.')
        parser.add_argument(
            '--compression-mode', type=str, default='none',
            choices=['none', 'file', 'message'],
            help='Choose mode of compression for the storage. Default: %(default)s.')
        parser.add_argument(
            '--compression-format', type=str, default='',
            choices=get_registered_compressors(),
            help='Choose the compression format/algorithm. '
                 'Has no effect if no compression mode is chosen. Default: %(default)s.')

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

        if args.compression_queue_size < 0:
            return print_error('Compression queue size must be at least 0.')

        args.compression_mode = args.compression_mode.upper()

        qos_profile_overrides = {}  # Specify a valid default
        if args.qos_profile_overrides_path:
            qos_profile_dict = yaml.safe_load(args.qos_profile_overrides_path)
            try:
                qos_profile_overrides = convert_yaml_to_qos_profile(
                    qos_profile_dict)
            except (InvalidQoSProfileException, ValueError) as e:
                return print_error(str(e))

        # Prepare custom_data dictionary
        custom_data = {}
        if args.custom_data:
            key_value_pairs = [pair.split('=') for pair in args.custom_data]
            custom_data = {pair[0]: pair[1] for pair in key_value_pairs}

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
            snapshot_mode=args.snapshot_mode,
            custom_data=custom_data
        )
        record_options = RecordOptions()
        record_options.all = args.all
        record_options.is_discovery_disabled = args.no_discovery
        record_options.topics = args.topics
        record_options.rmw_serialization_format = args.serialization_format
        record_options.topic_polling_interval = datetime.timedelta(
            milliseconds=args.polling_interval)
        record_options.regex = args.regex
        record_options.exclude = args.exclude
        record_options.node_prefix = NODE_NAME_PREFIX
        record_options.compression_mode = args.compression_mode
        record_options.compression_format = args.compression_format
        record_options.compression_queue_size = args.compression_queue_size
        record_options.compression_threads = args.compression_threads
        record_options.topic_qos_profile_overrides = qos_profile_overrides
        record_options.include_hidden_topics = args.include_hidden_topics
        record_options.include_unpublished_topics = args.include_unpublished_topics
        record_options.start_paused = args.start_paused
        record_options.ignore_leaf_topics = args.ignore_leaf_topics
        record_options.use_sim_time = args.use_sim_time

        recorder = Recorder()

        try:
            recorder.record(storage_options, record_options, args.node_name)
        except KeyboardInterrupt:
            pass

        if os.path.isdir(uri) and not os.listdir(uri):
            os.rmdir(uri)
