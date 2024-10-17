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
        parser.add_argument(
            '-t', '--topic-name', action='store_true',
            help='Only display topic names.'
        )
        parser.add_argument(
            '-v', '--verbose', action='store_true',
            help='Display request/response information for services'
        )
        available_sorting_methods = Info().get_sorting_methods()
        parser.add_argument(
            '--sort', default='name', choices=available_sorting_methods,
            help='Configuration for sorting of output. '
                 'By default sorts by topic name - use this argument to override.')

    def main(self, *, args):  # noqa: D102
        if args.topic_name and args.verbose:
            print("Warning! You have set both the '-t' and '-v' parameters. The '-t' parameter "
                  'will be ignored.')
        m = Info().read_metadata(args.bag_path, args.storage)
        if args.verbose:
            Info().print_output_verbose(args.bag_path, m, args.sort)
        else:
            if args.topic_name:
                Info().print_output_topic_name_only(m, args.sort)
            else:
                Info().print_output(m, args.sort)
