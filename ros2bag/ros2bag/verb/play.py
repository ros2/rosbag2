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

from rclpy.qos import InvalidQoSProfileException
from ros2bag.api import check_path_exists
from ros2bag.api import check_positive_float
from ros2bag.api import convert_yaml_to_qos_profile
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension
from ros2cli.node import NODE_NAME_PREFIX
import yaml
import sys


class PlayVerb(VerbExtension):
    """Play back ROS data from a bag."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            'bag_file', type=check_path_exists, help='bag file to replay')
        parser.add_argument(
            '-s', '--storage', default='sqlite3',
            help="storage identifier to be used, defaults to 'sqlite3'")
        parser.add_argument(
            '--read-ahead-queue-size', type=int, default=1000,
            help='size of message queue rosbag tries to hold in memory to help deterministic '
                 'playback. Larger size will result in larger memory needs but might prevent '
                 'delay of message playback.')
        parser.add_argument(
            '-r', '--rate', type=check_positive_float, default=1.0,
            help='rate at which to play back messages. Valid range > 0.0.')
        parser.add_argument(
            '--start_time', type=int, default=1,
            help='start timestamp from which to start playing the bag')
        parser.add_argument(
            '--stop_time', type=int, default=sys.maxsize,
            help='stop timestamp after which to stop playing the bag')
        parser.add_argument(
            '--topics', type=str, default='', nargs='+',
            help='topics to replay, separated by space. If none specified, all topics will be '
                 'replayed.')
        parser.add_argument(
            '--qos-profile-overrides-path', type=FileType('r'),
            help='Path to a yaml file defining overrides of the QoS profile for specific topics.')
        parser.add_argument(
            '-l', '--loop', action='store_true',
            help='enables loop playback when playing a bagfile: it starts back at the beginning '
                 'on reaching the end and plays indefinitely.')
        parser.add_argument(
            '--remap', '-m', default='', nargs='+',
            help='list of topics to be remapped: in the form '
                 '"old_topic1:=new_topic1 old_topic2:=new_topic2 etc." ')
        parser.add_argument(
            '--storage-config-file', type=FileType('r'),
            help='Path to a yaml file defining storage specific configurations. '
                 'For the default storage plugin settings are specified through syntax:'
                 'read:'
                 '  pragmas: [\"<setting_name>\" = <setting_value>]'
                 'Note that applicable settings are limited to read-only for ros2 bag play.'
                 'For a list of sqlite3 settings, refer to sqlite3 documentation')

    def main(self, *, args):  # noqa: D102
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

        # NOTE(hidmic): in merged install workspaces on Windows, Python entrypoint lookups
        #               combined with constrained environments (as imposed by colcon test)
        #               may result in DLL loading failures when attempting to import a C
        #               extension. Therefore, do not import rosbag2_transport at the module
        #               level but on demand, right before first use.
        from rosbag2_transport import rosbag2_transport_py
        rosbag2_transport_py.play(
            uri=args.bag_file,
            storage_id=args.storage,
            node_prefix=NODE_NAME_PREFIX,
            read_ahead_queue_size=args.read_ahead_queue_size,
            rate=args.rate,
            start_time=args.start_time,
            stop_time=args.stop_time,
            topics=args.topics,
            qos_profile_overrides=qos_profile_overrides,
            loop=args.loop,
            topic_remapping=args.remap,
            storage_config_file=storage_config_file)
