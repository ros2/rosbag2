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

from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension


class BagCommand(CommandExtension):
    """Various rosbag related sub-commands."""

    FUTURE_NOTICE = 'NOTE: Running rosbag2 "futurized" source release, not the official Foxy binary version.'  # NOQA

    def add_arguments(self, parser, cli_name):
        self._subparser = parser

        # get verb extensions and let them add their arguments
        add_subparsers_on_demand(
            parser, cli_name, '_verb', 'ros2bag.verb', required=False)

    def main(self, *, parser, args):
        print(self.FUTURE_NOTICE)
        print()
        if not hasattr(args, '_verb'):
            # in case no verb was passed
            self._subparser.print_help()
            return 0

        extension = getattr(args, '_verb')

        # call the verb's main method
        return extension.main(args=args)
