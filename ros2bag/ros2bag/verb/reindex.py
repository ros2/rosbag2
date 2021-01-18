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

import os

from ros2bag.api import check_path_exists
from ros2bag.verb import VerbExtension


class ReindexVerb(VerbExtension):
    """Generate metadata from a bag."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            'bag_file', type=check_path_exists, help='Bag file to reindex')
        parser.add_argument(
            '-s', '--storage', default='sqlite3',
            help="storage identifier to be used, defaults to 'sqlite3'")
        parser.add_argument(
            '-f', '--serialization-format', default='',
            help='rmw serialization format in which the messages are saved, defaults to the'
                 ' rmw currently in use')
        parser.add_argument(
            '--compression-format', type=str, default='', choices=['zstd'],
            help='Specify the compression format/algorithm. Default is none.'
        )
        self._subparser = parser

    def main(self, *, args):  # noqa: D102
        uri = args.bag_file

        # NOTE(hidmic): in merged install workspaces on Windows, Python entrypoint lookups
        #               combined with constrained environments (as imposed by colcon test)
        #               may result in DLL loading failures when attempting to import a C
        #               extension. Therefore, do not import rosbag2_transport at the module
        #               level but on demand, right before first use.
        from rosbag2_transport import rosbag2_transport_py

        rosbag2_transport_py.reindex(
            uri=uri,
            storage_id=args.storage,
            serialization_format=args.serialization_format,
            compression_format=args.compression_format
        )

        if os.path.isdir(uri) and not os.listdir(uri):
            os.rmdir(uri)
