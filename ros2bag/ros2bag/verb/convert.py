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

import argparse

from ros2bag.verb import VerbExtension
from rosbag2_py import bag_rewrite
from rosbag2_py import StorageOptions


class ConvertVerb(VerbExtension):
    """Given an input bag, write out a new bag with different settings."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-i', '--input',
            required=True,
            action='append', nargs='+',
            metavar=('uri', 'storage_id'),
            help='URI (and optional storage ID) of an input bag. May be provided more than once')
        parser.add_argument(
            '-o', '--output-options',
            type=str, required=True,
            help='YAML file with options for output bags. ')

    def main(self, *, args):
        input_options = []
        for input_bag in args.input:
            if len(input_bag) > 2:
                raise argparse.ArgumentTypeError(
                    f'--input expects 1 or 2 arguments, {len(input_bag)} provided')
            storage_options = StorageOptions()
            storage_options.uri = input_bag[0]
            if len(input_bag) > 1:
                storage_options.storage_id = input_bag[1]
            input_options.append(storage_options)

        bag_rewrite(input_options, args.output_options)
