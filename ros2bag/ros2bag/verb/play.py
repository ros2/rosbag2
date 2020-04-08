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

from ros2bag.api import check_path_exists
from ros2bag.api import check_positive_float
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension
from ros2cli.node import NODE_NAME_PREFIX


class PlayVerb(VerbExtension):
    """ros2 bag play."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            'bag_file', type=check_path_exists, help='bag file to replay')
        parser.add_argument(
            '-s', '--storage', default='sqlite3',
            help='storage identifier to be used, defaults to "sqlite3"')
        parser.add_argument(
            '--read-ahead-queue-size', type=int, default=1000,
            help='size of message queue rosbag tries to hold in memory to help deterministic '
                 'playback. Larger size will result in larger memory needs but might prevent '
                 'delay of message playback.')
        parser.add_argument(
            '-r', '--rate', type=check_positive_float, default=1.0,
            help='rate at which to play back messages. Valid range > 0.0.')

    def main(self, *, args):  # noqa: D102
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
            rate=args.rate)
