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

#include "../../rosbag2_cpp/test/rosbag2_cpp/mock_converter_factory.hpp"
#include "../../rosbag2_cpp/test/rosbag2_cpp/mock_metadata_io.hpp"
#include "../../rosbag2_cpp/test/rosbag2_cpp/mock_storage.hpp"
#include "../../rosbag2_cpp/test/rosbag2_cpp/mock_storage_factory.hpp"

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
      "topic", "test_msgs/BasicTypes", storage_serialization_format_};
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

  auto sequential_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));

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

  auto sequential_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));

  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
  // Throws runtime_error b/c compressor can't read
  // TODO(piraka9011): Use a compression factory in reader.
  EXPECT_THROW(
    reader_->open(rosbag2_cpp::StorageOptions(), {"", storage_serialization_format_}),
    std::runtime_error);
}
