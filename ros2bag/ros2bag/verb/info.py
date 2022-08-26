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

from ros2bag.api import add_standard_reader_args
from ros2bag.verb import VerbExtension
from rosbag2_py._info import Info


class InfoVerb(VerbExtension):
    """Print information about a bag to the screen."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_standard_reader_args(parser)

    def main(self, *, args):  # noqa: D102
        m = Info().read_metadata(args.bag_path, args.storage)
        print(m)
