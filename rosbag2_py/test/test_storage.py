# Copyright 2022, Foxglove Technologies. All rights reserved.
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

import unittest

import common  # noqa

from rclpy.duration import Duration
from rclpy.time import Time
from rosbag2_py import (
    BagMetadata,
    ConverterOptions,
    FileInformation,
    StorageFilter,
    StorageOptions,
    TopicInformation,
    TopicMetadata,
)


class TestStorageStructs(unittest.TestCase):

    def test_bag_metadata_default_ctor(self):
        metadata = BagMetadata()
        assert metadata

    def test_converter_options_ctor(self):
        converter_options = ConverterOptions()
        assert converter_options

    def test_file_information_ctor(self):
        file_information = FileInformation(
            path='test_path',
            starting_time=Time(nanoseconds=1000),
            duration=Duration(),
            message_count=1234,
        )
        assert file_information

    def test_storage_options_ctor(self):
        storage_options = StorageOptions(uri='path')
        assert storage_options

    def test_storage_filter_ctor(self):
        storage_filter = StorageFilter()
        assert storage_filter

    def test_topic_metadata_ctor(self):
        topic_metadata = TopicMetadata(
            name='topic',
            type='msgs/Msg',
            serialization_format='format'
        )
        assert topic_metadata

    def test_topic_information_ctor(self):
        topic_information = TopicInformation(
            topic_metadata=TopicMetadata(
                name='topic',
                type='msgs/Msg',
                serialization_format='format'),
            message_count=10
        )
        assert topic_information

    def test_bag_metadata_ctor_named_args(self):
        duration = Duration(nanoseconds=200)
        starting_time = Time(nanoseconds=100)

        file_information = FileInformation(
            path='something',
            starting_time=starting_time,
            duration=duration,
            message_count=12)
        topic_information = TopicInformation(
            topic_metadata=TopicMetadata(
                name='topic',
                type='msgs/Msg',
                serialization_format='format'),
            message_count=10
        )

        metadata = BagMetadata(
            version=1,
            bag_size=2,
            storage_identifier='foo',
            relative_file_paths=['bar', 'baz'],
            files=[file_information],
            duration=duration,
            starting_time=starting_time,
            message_count=12,
            topics_with_message_count=[topic_information],
            compression_format='aaaa',
            compression_mode='bbbbb',
            custom_data={
                'keya': 'valuea',
                'keyb': 'valueb'
            }
        )
        assert metadata
