# Copyright 2020 AIT Austrian Institute of Technology GmbH
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

import datetime
import os
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension


def get_rosbag_options(path, serialization_format='cdr'):
    import rosbag2_py
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def determine_compression_mode(path):
    # TODO determine from metadata
    return 'none'


def compression_mode_from_string(mode):
    if mode == 'none':
        return 0
    if mode == 'file':
        return 1
    if mode == 'message':
        return 2
    raise ValueError(f'invalid compression mode: {mode}')


class ConvertVerb(VerbExtension):
    """ros2 bag convert."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            'bag_file', help='bag file to convert')
        parser.add_argument(
            '-o', '--output',
            help='destination of the bagfile to create, \
            defaults to a timestamped folder in the current directory')
        parser.add_argument(
            '-s', '--in-storage',
            help='storage identifier to be used for the input bag, defaults to "sqlite3"')
        parser.add_argument(
            '--out-storage', default='sqlite3',
            help='storage identifier to be used for the output bag, defaults to "sqlite3"')
        parser.add_argument(
            '--compression-mode', type=str, default='none',
            choices=['none', 'file', 'message'],
            help="Determine whether to compress by file or message. Default is 'none'.")
        parser.add_argument(
            '--compression-format', type=str, default='', choices=['zstd'],
            help='Specify the compression format/algorithm. Default is none.')
        parser.add_argument(
            '-f', '--serialization-format', default='',
            help='rmw serialization format in which the messages are saved, defaults to the'
                 ' rmw currently in use')

    def main(self, *, args):  # noqa: D102
        bag_file = args.bag_file
        if not os.path.exists(bag_file):
            return print_error("bag file '{}' does not exist!".format(bag_file))

        uri = args.output or datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')

        if os.path.isdir(uri):
            return print_error("Output folder '{}' already exists.".format(uri))

        if args.compression_format and args.compression_mode == 'none':
            return print_error('Invalid choice: Cannot specify compression format '
                               'without a compression mode.')

        # NOTE(hidmic): in merged install workspaces on Windows, Python entrypoint lookups
        #               combined with constrained environments (as imposed by colcon test)
        #               may result in DLL loading failures when attempting to import a C
        #               extension. Therefore, do not import rosbag2_transport at the module
        #               level but on demand, right before first use.
        from rosbag2_py import (
            SequentialReader,
            SequentialCompressionReader,
            SequentialWriter,
            SequentialCompressionWriter,
            StorageOptions,
            ConverterOptions,
            # CompressionOptions,
        )

        if determine_compression_mode(bag_file) == 'none':
            reader = SequentialReader()
        else:
            reader = SequentialCompressionReader()

        in_storage_options, in_converter_options = get_rosbag_options(bag_file)
        if args.in_storage:
            in_storage_options.storage = args.in_storage
        reader.open(in_storage_options, in_converter_options)

        # out_compression_options = CompressionOptions()
        # out_compression_options.compression_format = args.compression_format
        # out_compression_options.compression_mode = compression_mode_from_string(args.compression_mode)

        if args.compression_mode != 'none':
            writer = SequentialCompressionWriter()
        else:
            writer = SequentialWriter()

        out_storage_options = StorageOptions(uri=uri, storage_id=args.out_storage)
        out_converter_options = ConverterOptions(
            input_serialization_format=args.serialization_format,
            output_serialization_format=args.serialization_format)
        writer.open(out_storage_options, out_converter_options)

        for topic_metadata in reader.get_all_topics_and_types():
            writer.create_topic(topic_metadata)

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            writer.write(topic, data, t)

        del writer
        del reader

        if os.path.isdir(uri) and not os.listdir(uri):
            os.rmdir(uri)
