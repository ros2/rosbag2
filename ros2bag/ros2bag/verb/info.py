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

    def _is_service_event_topic(self, topic_name, topic_type) -> bool:

        service_event_type_middle = '/srv/'
        service_event_type_postfix = '_Event'

        if (service_event_type_middle not in topic_type
                or not topic_type.endswith(service_event_type_postfix)):
            return False

        service_event_topic_postfix = '/_service_event'
        if not topic_name.endswith(service_event_topic_postfix):
            return False

        return True

    def main(self, *, args):  # noqa: D102
        if args.topic_name and args.verbose:
            print("Warning! You have set both the '-t' and '-v' parameters. The '-t' parameter "
                  'will be ignored.')
        if args.verbose:
            Info().read_metadata_and_output_service_verbose(args.bag_path, args.storage)
        else:
            m = Info().read_metadata(args.bag_path, args.storage)
            if args.topic_name:
                for topic_info in m.topics_with_message_count:
                    if not self._is_service_event_topic(topic_info.topic_metadata.name,
                                                        topic_info.topic_metadata.type):
                        print(topic_info.topic_metadata.name)
            else:
                print(m)
