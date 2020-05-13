// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_compression/sequential_compression_reader.hpp"

#include "rosbag2_cpp/reader.hpp"

#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

#include "mock_compression.hpp"
#include "mock_compression_factory.hpp"

using namespace testing;  // NOLINT

class SequentialCompressionReaderTest : public Test
{
public:
  SequentialCompressionReaderTest()
  : storage_factory_{std::make_unique<StrictMock<MockStorageFactory>>()},
    storage_{std::make_shared<NiceMock<MockStorage>>()},
    converter_factory_{std::make_shared<StrictMock<MockConverterFactory>>()},
    metadata_io_{std::make_unique<NiceMock<MockMetadataIo>>()},
    storage_serialization_format_{"rmw1_format"}
  {
    topic_with_type_ = rosbag2_storage::TopicMetadata{
      "topic", "test_msgs/BasicTypes", storage_serialization_format_, ""};
    auto topics_and_types = std::vector<rosbag2_storage::TopicMetadata>{topic_with_type_};
    auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    message->topic_name = topic_with_type_.name;

    ON_CALL(*storage_, get_all_topics_and_types()).WillByDefault(Return(topics_and_types));
    ON_CALL(*storage_, read_next()).WillByDefault(Return(message));
    ON_CALL(*storage_factory_, open_read_only(_, _)).WillByDefault(Return(storage_));
  }

  std::unique_ptr<StrictMock<MockStorageFactory>> storage_factory_;
  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<NiceMock<MockMetadataIo>> metadata_io_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::string storage_serialization_format_;
  rosbag2_storage::TopicMetadata topic_with_type_;
};

TEST_F(SequentialCompressionReaderTest, open_throws_if_unsupported_compressor)
{
  rosbag2_storage::BagMetadata metadata;
  metadata.relative_file_paths = {"/path/to/storage"};
  metadata.topics_with_message_count.push_back({{topic_with_type_}, 1});
  metadata.compression_format = "bad_format";
  metadata.compression_mode =
    rosbag2_compression::compression_mode_to_string(rosbag2_compression::CompressionMode::FILE);
  EXPECT_CALL(*metadata_io_, read_metadata(_)).WillRepeatedly(Return(metadata));
  EXPECT_CALL(*metadata_io_, metadata_file_exists(_)).WillRepeatedly(Return(true));
  auto compression_factory = std::make_unique<rosbag2_compression::CompressionFactory>();

  auto sequential_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(compression_factory),
    std::move(storage_factory_),
    converter_factory_,
    std::move(metadata_io_));

  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
  EXPECT_THROW(
    reader_->open(rosbag2_cpp::StorageOptions(), {"", storage_serialization_format_}),
    std::invalid_argument);
}

TEST_F(SequentialCompressionReaderTest, open_supports_zstd_compressor)
{
  rosbag2_storage::BagMetadata metadata;
  metadata.relative_file_paths = {"/path/to/storage"};
  metadata.topics_with_message_count.push_back({{topic_with_type_}, 1});
  metadata.compression_format = "zstd";
  metadata.compression_mode =
    rosbag2_compression::compression_mode_to_string(rosbag2_compression::CompressionMode::FILE);
  ON_CALL(*metadata_io_, read_metadata(_)).WillByDefault(Return(metadata));
  ON_CALL(*metadata_io_, metadata_file_exists(_)).WillByDefault(Return(true));
  auto compression_factory = std::make_unique<rosbag2_compression::CompressionFactory>();

  auto sequential_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(compression_factory),
    std::move(storage_factory_),
    converter_factory_,
    std::move(metadata_io_));

  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
  // Throws runtime_error b/c compressor can't read
  EXPECT_THROW(
    reader_->open(rosbag2_cpp::StorageOptions(), {"", storage_serialization_format_}),
    std::runtime_error);
}

TEST_F(SequentialCompressionReaderTest, reader_calls_create_decompressor)
{
  rosbag2_storage::BagMetadata metadata;
  metadata.relative_file_paths = {"/path/to/storage"};
  metadata.topics_with_message_count.push_back({{topic_with_type_}, 1});
  metadata.compression_format = "zstd";
  metadata.compression_mode =
    rosbag2_compression::compression_mode_to_string(rosbag2_compression::CompressionMode::FILE);
  ON_CALL(*metadata_io_, read_metadata(_)).WillByDefault(Return(metadata));
  ON_CALL(*metadata_io_, metadata_file_exists(_)).WillByDefault(Return(true));

  auto decompressor = std::make_unique<NiceMock<MockDecompressor>>();
  ON_CALL(*decompressor, decompress_uri(_)).WillByDefault(Return("some/path"));
  EXPECT_CALL(*decompressor, decompress_uri(_)).Times(1);

  auto compression_factory = std::make_unique<StrictMock<MockCompressionFactory>>();
  ON_CALL(*compression_factory, create_decompressor(_))
  .WillByDefault(Return(ByMove(std::move(decompressor))));
  EXPECT_CALL(*compression_factory, create_decompressor(_)).Times(1);
  EXPECT_CALL(*storage_factory_, open_read_only(_, _)).Times(1);

  auto sequential_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(compression_factory),
    std::move(storage_factory_),
    converter_factory_,
    std::move(metadata_io_));

  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
  reader_->open(
    rosbag2_cpp::StorageOptions(), {"", storage_serialization_format_});
}

TEST_F(SequentialCompressionReaderTest, compression_called_when_splitting_bagfile)
{
  const auto relative_path_1 = "/path/to/storage1";
  const auto relative_path_2 = "/path/to/storage2";
  rosbag2_storage::BagMetadata metadata;
  metadata.relative_file_paths = {relative_path_1, relative_path_2};
  metadata.topics_with_message_count.push_back({{topic_with_type_}, 10});
  metadata.bag_size = 512000;
  metadata.compression_format = "zstd";
  metadata.compression_mode =
    rosbag2_compression::compression_mode_to_string(rosbag2_compression::CompressionMode::FILE);
  ON_CALL(*metadata_io_, read_metadata(_)).WillByDefault(Return(metadata));
  ON_CALL(*metadata_io_, metadata_file_exists(_)).WillByDefault(Return(true));

  auto decompressor = std::make_unique<NiceMock<MockDecompressor>>();
  // We are mocking two splits, so only file decompression should occur twice
  EXPECT_CALL(*decompressor, decompress_uri(_)).Times(2)
  .WillOnce(Return(relative_path_1))
  .WillOnce(Return(relative_path_2));
  EXPECT_CALL(*decompressor, decompress_serialized_bag_message(_)).Times(0);

  auto compression_factory = std::make_unique<StrictMock<MockCompressionFactory>>();
  ON_CALL(*compression_factory, create_decompressor(_))
  .WillByDefault(Return(ByMove(std::move(decompressor))));
  EXPECT_CALL(*compression_factory, create_decompressor(_)).Times(1);
  // open_read_only should be called twice when opening 2 split bags
  EXPECT_CALL(*storage_factory_, open_read_only(_, _)).Times(2);
  // storage::has_next() is called twice when reader::has_next() is called
  EXPECT_CALL(*storage_, has_next()).Times(2)
  .WillOnce(Return(false))  // Load the next file
  .WillOnce(Return(true));  // We have a message from the new file

  auto compression_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(compression_factory),
    std::move(storage_factory_),
    converter_factory_,
    std::move(metadata_io_));

  compression_reader->open(
    rosbag2_cpp::StorageOptions(), {"", storage_serialization_format_});
  EXPECT_EQ(compression_reader->has_next_file(), true);
  EXPECT_EQ(compression_reader->has_next(), true);
  compression_reader->read_next();
}
