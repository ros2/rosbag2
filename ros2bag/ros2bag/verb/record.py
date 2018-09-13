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

from ros2bag.verb import VerbExtension

from ros2cli.node.strategy import NodeStrategy
from ros2cli.node.strategy import add_arguments

from rosbag2_transport import rosbag2_transport_py


class RecordVerb(VerbExtension):
    """ros2 bag record."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        self._subparser = parser

        add_arguments(parser)
        parser.add_argument(
            '-a', '--all', action='store_true', help='recording all topics')
        parser.add_argument(
            'topics', nargs='*', help='topics to be recorded')

    def main(self, *, args):  # noqa: D102
        if args.all and args.topics:
            print('invalid choice: Can not specify topics and -a at the same time')
            return

        if args.all:
            rosbag2_transport_py.record_topics([])
        elif args.topics and len(args.topics) > 0:
            rosbag2_transport_py.record_topics(args.topics)
        else:
            self._subparser.print_help()
