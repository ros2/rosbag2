# Copyright 2020 Open Source Robotics Foundation, Inc.
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

from rpyutils import add_dll_directories_from_env

# Since Python 3.8, on Windows we should ensure DLL directories are explicitly added
# to the search path.
# See https://docs.python.org/3/whatsnew/3.8.html#bpo-36085-whatsnew
with add_dll_directories_from_env('PATH'):
    from rosbag2_py._reader import (
        SequentialCompressionReader,
        SequentialReader,
        get_registered_readers,
    )
    from rosbag2_py._storage import (
        ConverterOptions,
        StorageFilter,
        StorageOptions,
        TopicMetadata,
        TopicInformation,
        BagMetadata,
    )
    from rosbag2_py._writer import (
        SequentialCompressionWriter,
        SequentialWriter,
        get_registered_writers,
    )
    from rosbag2_py._info import (
        Info,
    )

__all__ = [
    'ConverterOptions',
    'get_registered_readers',
    'get_registered_writers',
    'SequentialCompressionReader',
    'SequentialCompressionWriter',
    'SequentialReader',
    'SequentialWriter',
    'StorageFilter',
    'StorageOptions',
    'TopicMetadata',
    'TopicInformation',
    'BagMetadata',
    'Info',
]
