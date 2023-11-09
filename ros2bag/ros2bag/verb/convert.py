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
from rosbag2_py import (
    bag_rewrite,
    RecordOptions,
    StorageOptions,
)
import yaml


def input_bag_options(config: str):
    return StorageOptions(**yaml.safe_load(config))


def output_bag_options(config: str):
    kwargs = yaml.safe_load(config)
    storage_options = StorageOptions(uri=kwargs['uri'])
    record_options = RecordOptions()
    for key, val in kwargs.items():
        try:
            storage_options.__setattr__(key, val)
            continue
        except AttributeError:
            pass
        try:
            record_options.__setattr__(key, val)
            continue
        except AttributeError:
            pass
        raise argparse.ArgumentError(f'Unknown output option "{key}"')

    return storage_options, record_options


class ConvertVerb(VerbExtension):
    """Given an input bag, write out a new bag with different settings."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-i', '--input',
            required=True,
            type=input_bag_options,
            action='append',
            help='StorageOptions as YAML string.',
        )
        parser.add_argument(
            '-o', '--output',
            required=True,
            type=output_bag_options,
            action='append',
            help='Combined StorageOptions and RecordOptions as YAML string.',
        )

    def main(self, *, args):
        bag_rewrite(args.input, args.output)
