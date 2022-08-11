# Copyright 2021 DCS Corporation, All Rights Reserved.
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
#
# DISTRIBUTION A. Approved for public release; distribution unlimited.
# OPSEC #4584.
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS
# Part 252.227-7013 or 7014 (Feb 2014).
#
# This notice must appear in all copies of this file and its derivatives.

import os

from ros2bag.api import (
    add_standard_reader_args,
    print_error,
)
from ros2bag.verb import VerbExtension
from rosbag2_py import Reindexer, StorageOptions


class ReindexVerb(VerbExtension):
    """Reconstruct metadata file for a bag."""

    def add_arguments(self, parser, cli_name):
        add_standard_reader_args(parser)

    def main(self, *, args):
        if not os.path.isdir(args.bag_path):
            return print_error('Must specify a bag directory')

        storage_options = StorageOptions(
            uri=args.bag_path,
            storage_id=args.storage_id,
        )

        reindexer = Reindexer()
        reindexer.reindex(storage_options)
