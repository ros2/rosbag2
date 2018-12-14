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

import os
import sys

from ros2bag.verb import VerbExtension

from rosbag2_transport import rosbag2_transport_py


class PlayVerb(VerbExtension):
    """ros2 bag play."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            'bag_file', help='bag file to replay')
        parser.add_argument(
            '-s', '--storage', default='sqlite3',
            help='storage identifier to be used, defaults to "sqlite3"')
        parser.add_argument(
            '-r', '--read-ahead-queue-size', default = 1000,
            help='size of message queue rosbag tries to hold in memory to help deterministic '
                 'playback. Larger size will result in larger memory needs but might prevent '
                 'delay of message playback.')

    def main(self, *, args):  # noqa: D102
        bag_file = args.bag_file
        if not os.path.exists(bag_file):
            return "Error: bag file '{}' does not exist!".format(bag_file)

        read_ahead_queue_size = 1000
        try:
            read_ahead_queue_size = int(args.read_ahead_queue_size)
        except:
            print("[ERROR] [ros2bag] read ahead queue size must be an integer")
            exit(0)

        rosbag2_transport_py.play(
            uri=bag_file,
            storage_id=args.storage,
            read_ahead_queue_size=read_ahead_queue_size)
