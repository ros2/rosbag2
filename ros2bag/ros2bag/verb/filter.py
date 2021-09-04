# Copyright 2021 Tier IV, Inc. All rights reserved.
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

from ros2bag.api import check_path_exists
from ros2bag.verb import VerbExtension
from rosbag2_py import get_registered_readers
from rosbag2_py import ConverterOptions
from rosbag2_py import Reindexer
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions


class FilterVerb(VerbExtension):
    """Filter by python expression."""

    def _bag2filter(self, input_bag_dir: str, output_bag_dir: str,
                    python_expr: str, storage_id: str) -> None:
        storage_options = StorageOptions(
            uri=input_bag_dir,
            storage_id=storage_id,
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        writer = SequentialWriter()
        storage_options = StorageOptions(
            uri=output_bag_dir,
            storage_id=storage_id,
        )
        writer.open(storage_options, converter_options)

        for topic_type in reader.get_all_topics_and_types():
            writer.create_topic(topic_type)

        filter_fn = eval('lambda topic, m, t: %s' % python_expr)

        while reader.has_next():
            topic_name, msg, stamp = reader.read_next()
            if filter_fn(topic_name, msg, stamp):
                writer.write(topic_name, msg, stamp)
        del writer

        Reindexer().reindex(storage_options)

    def add_arguments(self, parser, cli_name):
        storage_choices = get_registered_readers()
        default_storage = 'sqlite3' if 'sqlite3' in storage_choices else storage_choices[0]
        parser.add_argument(
            'bag_directory', type=check_path_exists, help='Bag to filter')
        parser.add_argument(
            'output_direcory', help='Output directory')
        parser.add_argument(
            'python_expression', help='Python expression to filter')
        parser.add_argument(
            '-s', '--storage_id', default=default_storage, choices=storage_choices,
            help=f'storage identifier to be used, defaults to {default_storage}')

    def main(self, *, args):
        if os.path.isdir(args.output_direcory):
            raise FileExistsError(f'Output folder "{args.output_direcory}" already exists.')

        self._bag2filter(args.bag_directory, args.output_direcory,
                         args.python_expression, args.storage_id)
