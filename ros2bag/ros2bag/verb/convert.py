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

import datetime
import os

from ros2bag.verb import VerbExtension


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
            '-s', '--storage', default='sqlite3',
            help='storage identifier to be used for the input bag, defaults to "sqlite3"')
        parser.add_argument(
            '--out-storage', default='sqlite3',
            help='storage identifier to be used for the output bag, defaults to "sqlite3"')
        parser.add_argument(
            '-f', '--serialization-format', default='',
            help='rmw serialization format in which the messages are saved, defaults to the'
                 ' rmw currently in use')
        parser.add_argument(
            '--compression-mode', type=str, default='none',
            choices=['none', 'file', 'message'],
            help='Determine whether to compress by file or message. Default is "none".')
        parser.add_argument(
            '--compression-format', type=str, default='', choices=['zstd'],
            help='Specify the compression format/algorithm. Default is none.')

    def create_bag_directory(self, uri):
        try:
            os.makedirs(uri)
        except OSError:
            return "[ERROR] [ros2bag]: Could not create bag folder '{}'.".format(uri)

    def main(self, *, args):  # noqa: D102
        bag_file = args.bag_file
        if not os.path.exists(bag_file):
            return "[ERROR] [ros2bag] bag file '{}' does not exist!".format(bag_file)

        if args.compression_format and args.compression_mode == 'none':
            return 'Invalid choice: Cannot specify compression format without a compression mode.'
        args.compression_mode = args.compression_mode.upper()

        uri = args.output or datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')
        self.create_bag_directory(uri)

        # NOTE(hidmic): in merged install workspaces on Windows, Python entrypoint lookups
        #               combined with constrained environments (as imposed by colcon test)
        #               may result in DLL loading failures when attempting to import a C
        #               extension. Therefore, do not import rosbag2_transport at the module
        #               level but on demand, right before first use.
        from rosbag2_transport import rosbag2_transport_py

        rosbag2_transport_py.convert(
            in_uri=bag_file,
            in_storage_id=args.storage,
            out_uri=uri,
            out_storage_id=args.out_storage,
            serialization_format=args.serialization_format,
            compression_mode=args.compression_mode,
            compression_format=args.compression_format)

        if os.path.isdir(uri) and not os.listdir(uri):
            os.rmdir(uri)
