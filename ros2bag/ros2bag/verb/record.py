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

import datetime
import os

from ros2bag.verb import VerbExtension

from ros2cli.node import NODE_NAME_PREFIX


class RecordVerb(VerbExtension):
    """ros2 bag record."""

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
            help='storage identifier to be used, defaults to "sqlite3"')
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
            help='time in ms to wait between querying available topics for recording. It has no '
                 'effect if --no-discovery is enabled.'
        )
        self._subparser = parser

    def create_bag_directory(self, uri):
        try:
            os.makedirs(uri)
        except OSError:
            return "[ERROR] [ros2bag]: Could not create bag folder '{}'.".format(uri)

    def main(self, *, args):  # noqa: D102
        if args.all and args.topics:
            return 'Invalid choice: Can not specify topics and -a at the same time.'

        uri = args.output or datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')

        if os.path.isdir(uri):
            return "[ERROR] [ros2bag]: Output folder '{}' already exists.".format(uri)

        self.create_bag_directory(uri)

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
                all=True,
                no_discovery=args.no_discovery,
                polling_interval=args.polling_interval)
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
                no_discovery=args.no_discovery,
                polling_interval=args.polling_interval,
                topics=args.topics)
        else:
            self._subparser.print_help()

        if os.path.isdir(uri) and not os.listdir(uri):
            os.rmdir(uri)
