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

    def main(self, *, args):  # noqa: D102
        bag_file = args.bag_file
        if not os.path.exists(bag_file):
            sys.exit("Error: bag file '{}' does not exist!".format(bag_file))

        rosbag2_transport_py.play(uri=bag_file, storage_id='sqlite3')
