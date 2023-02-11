# Copyright 2023 Foxglove Technologies Inc. All Rights Reserved.
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
from typing import List, Tuple

import yaml


def get_preset_profiles() -> List[Tuple[str, str]]:
    """Return list of pairs (profile_name, profile_description)."""
    return [
        ('none', 'Default profile, no special settings.'),
        ('fastwrite', 'Disables CRC and chunking for faster writing.'),
        ('zstd_fast', 'Use Zstd chunk compression on Fastest level.'),
        ('zstd_small', 'Use Zstd chunk compression on Slowest level, for smallest file size.'),
    ]


def add_custom_writer_arguments(parser: argparse.ArgumentParser) -> None:
    """Add storage-specific file-writing CLI arguments to the parser."""
    parser.add_argument('--no-chunking', action='store_true')
    parser.add_argument('--no-summary-crc', action='store_true')
    parser.add_argument('--no-chunk-crc', action='store_true')
    parser.add_argument('--chunk-size', type=int)


def parse_custom_writer_arguments(args: argparse.Namespace) -> str:
    """Read the output from storage-added arguments, output contents of a storage config file."""
    config = {}
    if args.storage_config_file:
        with open(args.storage_config_file, 'r') as f:
            config = yaml.safe_load(f)

    # For optional arguments, only overwrite provided value if actually specified on CLI
    if args.no_chunk_crc:
        config['noChunkCRC'] = True
    if args.no_summary_crc:
        config['noSummaryCRC'] = True
    if args.no_chunking:
        config['noChunking'] = True
    if args.chunk_size is not None:
        config['chunkSize'] = args.chunk_size

    return yaml.dumps(config)
