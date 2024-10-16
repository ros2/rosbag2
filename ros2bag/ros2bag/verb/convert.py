# Copyright 2021 Amazon.com Inc or its Affiliates
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

from ros2bag.api import add_multi_bag_input_arg
from ros2bag.api import input_bag_arg_to_storage_options
from ros2bag.verb import VerbExtension
from rosbag2_py import bag_rewrite


class ConvertVerb(VerbExtension):
    """Given an input bag, write out a new bag with different settings."""

    def add_arguments(self, parser, cli_name):
        add_multi_bag_input_arg(parser, required=True)
        parser.add_argument(
            '-o', '--output-options',
            type=str, required=True,
            help='YAML file with options for output bags. Must have one top-level key '
                 '"output_bags", which contains a sequence of StorageOptions/RecordOptions '
                 'objects. See README.md for some examples.')

    def main(self, *, args):
        input_options = input_bag_arg_to_storage_options(args.input)

        bag_rewrite(input_options, args.output_options)
