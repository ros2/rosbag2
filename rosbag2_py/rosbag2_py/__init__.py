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
    from rosbag2_py._compression_options import (
        CompressionMode,
        CompressionOptions,
        compression_mode_from_string,
        compression_mode_to_string
    )
    from rosbag2_py._message_definitions import (
        LocalMessageDefinitionSource,
    )
    from rosbag2_py._reader import (
        SequentialCompressionReader,
        SequentialReader,
        get_registered_readers,
    )
    from rosbag2_py._storage import (
        BagMetadata,
        ConverterOptions,
        FileInformation,
        MessageDefinition,
        MetadataIo,
        ReadOrder,
        ReadOrderSortBy,
        StorageFilter,
        StorageOptions,
        TopicMetadata,
        TopicInformation,
        convert_rclcpp_qos_to_rclpy_qos,
        get_default_storage_id,
        to_rclcpp_qos_vector,
    )
    from rosbag2_py._writer import (
        SequentialCompressionWriter,
        SequentialWriter,
        get_registered_writers,
        get_registered_compressors,
        get_registered_serializers,
    )
    from rosbag2_py._info import (
        Info,
    )
    from rosbag2_py._transport import (
        Player,
        PlayOptions,
        ServiceRequestsSource,
        Recorder,
        RecordOptions,
        bag_rewrite,
    )
    from rosbag2_py._reindexer import (
        Reindexer
    )

__all__ = [
    'bag_rewrite',
    'convert_rclcpp_qos_to_rclpy_qos',
    'CompressionMode',
    'CompressionOptions',
    'compression_mode_from_string',
    'compression_mode_to_string',
    'ConverterOptions',
    'FileInformation',
    'get_default_storage_id',
    'get_registered_readers',
    'get_registered_writers',
    'get_registered_compressors',
    'get_registered_serializers',
    'to_rclcpp_qos_vector',
    'ReadOrder',
    'ReadOrderSortBy',
    'Reindexer',
    'SequentialCompressionReader',
    'SequentialCompressionWriter',
    'SequentialReader',
    'SequentialWriter',
    'StorageFilter',
    'StorageOptions',
    'TopicMetadata',
    'TopicInformation',
    'BagMetadata',
    'MessageDefinition',
    'MetadataIo',
    'Info',
    'Player',
    'PlayOptions',
    'ServiceRequestsSource',
    'Recorder',
    'RecordOptions',
    'LocalMessageDefinitionSource',
]
