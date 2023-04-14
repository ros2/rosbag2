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

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/asserts.hpp"
#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_compression/sequential_compression_reader.hpp"

#include "rosbag2_cpp/reader.hpp"

#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

#include "mock_compression.hpp"
#include "mock_compression_factory.hpp"

using namespace testing;  // NOLINT

static constexpr const char * DefaultTestCompressor = "fake_comp";

class SequentialCompressionReaderTest : public Test
{
public:
  SequentialCompressionReaderTest()
  : storage_factory_{std::make_unique<NiceMock<MockStorageFactory>>()},
    storage_{std::make_shared<NiceMock<MockStorage>>()},
    converter_factory_{std::make_shared<StrictMock<MockConverterFactory>>()},
    metadata_io_{std::make_unique<NiceMock<MockMetadataIo>>()},
    storage_serialization_format_{"rmw1_format"},
    tmp_dir_{rcpputils::fs::temp_directory_path() / bag_name_},
    converter_options_{"", storage_serialization_format_}
  {
    rcpputils::fs::remove_all(tmp_dir_);
    storage_options_.uri = tmp_dir_.string();
    topic_with_type_ = rosbag2_storage::TopicMetadata{
      "topic", "test_msgs/BasicTypes", storage_serialization_format_, "", ""};
    auto topics_and_types = std::vector<rosbag2_storage::TopicMetadata>{topic_with_type_};
    auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    message->topic_name = topic_with_type_.name;

    metadata_ = construct_default_bag_metadata();
    ON_CALL(*metadata_io_, read_metadata).WillByDefault(
      [this](auto /*uri*/) {
        return metadata_;
      });
    ON_CALL(*metadata_io_, metadata_file_exists(_)).WillByDefault(Return(true));

    ON_CALL(*storage_, get_all_topics_and_types()).WillByDefault(Return(topics_and_types));
    ON_CALL(*storage_, read_next()).WillByDefault(Return(message));
    ON_CALL(*storage_, set_read_order).WillByDefault(Return(true));
    ON_CALL(*storage_factory_, open_read_only(_)).WillByDefault(Return(storage_));

    initialize_dummy_storage_files();
  }

  rosbag2_storage::BagMetadata construct_default_bag_metadata() const
  {
    rosbag2_storage::BagMetadata metadata;
    metadata.version = 4;
    metadata.relative_file_paths = {
      "bagfile_0." + std::string(DefaultTestCompressor),
      "bagfile_1." + std::string(DefaultTestCompressor)
    };
    metadata.topics_with_message_count.push_back({{topic_with_type_}, 1});
    metadata.compression_format = DefaultTestCompressor;
    metadata.compression_mode =
      rosbag2_compression::compression_mode_to_string(rosbag2_compression::CompressionMode::FILE);
    return metadata;
  }

  void initialize_dummy_storage_files()
  {
    // Initialize some dummy files so that they can be found
    rcpputils::fs::create_directories(tmp_dir_);
    for (auto relative : metadata_.relative_file_paths) {
      std::ofstream output((tmp_dir_ / relative).string());
      output << "Fake storage data" << std::endl;
    }
  }

  std::unique_ptr<rosbag2_compression::SequentialCompressionReader> create_reader()
  {
    auto decompressor = std::make_unique<NiceMock<MockDecompressor>>();
    ON_CALL(*decompressor, decompress_uri).WillByDefault(
      [](auto uri) {
        auto path = rcpputils::fs::path(uri);
        EXPECT_TRUE(path.exists());
        return rcpputils::fs::remove_extension(path).string();
      });
    auto compression_factory = std::make_unique<NiceMock<MockCompressionFactory>>();
    ON_CALL(*compression_factory, create_decompressor(_))
    .WillByDefault(Return(ByMove(std::move(decompressor))));
    return std::make_unique<rosbag2_compression::SequentialCompressionReader>(
      std::move(compression_factory),
      std::move(storage_factory_),
      converter_factory_,
      std::move(metadata_io_));
  }

  std::unique_ptr<MockStorageFactory> storage_factory_;
  std::shared_ptr<MockStorage> storage_;
  std::shared_ptr<MockConverterFactory> converter_factory_;
  std::unique_ptr<MockMetadataIo> metadata_io_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::string storage_serialization_format_;
  rosbag2_storage::TopicMetadata topic_with_type_;
  const std::string bag_name_ = "SequentialCompressionReaderTest";
  rcpputils::fs::path tmp_dir_;
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_storage::BagMetadata metadata_;
  rosbag2_cpp::ConverterOptions converter_options_;
};

TEST_F(SequentialCompressionReaderTest, open_throws_if_unsupported_compressor)
{
  metadata_.compression_format = "bad_format";
  EXPECT_CALL(*metadata_io_, read_metadata(_)).Times(1);
  EXPECT_CALL(*metadata_io_, metadata_file_exists(_)).Times(AtLeast(1));
  auto compression_factory = std::make_unique<rosbag2_compression::CompressionFactory>();

  auto sequential_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(compression_factory),
    std::move(storage_factory_),
    converter_factory_,
    std::move(metadata_io_));

  EXPECT_THROW(
    sequential_reader->open(storage_options_, converter_options_),
    rcpputils::IllegalStateException);
}

TEST_F(SequentialCompressionReaderTest, returns_all_topics_and_types)
{
  auto decompressor = std::make_unique<NiceMock<MockDecompressor>>();
  auto compression_factory = std::make_unique<StrictMock<MockCompressionFactory>>();

  ON_CALL(*compression_factory, create_decompressor(_))
  .WillByDefault(Return(ByMove(std::move(decompressor))));
  EXPECT_CALL(*compression_factory, create_decompressor(_)).Times(1);
  EXPECT_CALL(*storage_factory_, open_read_only(_)).Times(1);

  auto compression_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(compression_factory),
    std::move(storage_factory_),
    converter_factory_,
    std::move(metadata_io_));

  compression_reader->open(storage_options_, converter_options_);

  auto topics_and_types = compression_reader->get_all_topics_and_types();
  EXPECT_FALSE(topics_and_types.empty());
}

TEST_F(SequentialCompressionReaderTest, compressor_factory_creates_callable_compressor)
{
  auto compression_factory = std::make_unique<rosbag2_compression::CompressionFactory>();

  auto sequential_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(compression_factory),
    std::move(storage_factory_),
    converter_factory_,
    std::move(metadata_io_));

  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
  EXPECT_NO_THROW(reader_->open(storage_options_, converter_options_));
}

TEST_F(SequentialCompressionReaderTest, reader_calls_create_decompressor)
{
  auto decompressor = std::make_unique<NiceMock<MockDecompressor>>();
  ON_CALL(*decompressor, decompress_uri(_)).WillByDefault(Return("some/path"));
  EXPECT_CALL(*decompressor, decompress_uri(_)).Times(1);

  auto compression_factory = std::make_unique<StrictMock<MockCompressionFactory>>();
  ON_CALL(*compression_factory, create_decompressor(_))
  .WillByDefault(Return(ByMove(std::move(decompressor))));
  EXPECT_CALL(*compression_factory, create_decompressor(_)).Times(1);
  EXPECT_CALL(*storage_factory_, open_read_only(_)).Times(1);

  auto sequential_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(compression_factory),
    std::move(storage_factory_),
    converter_factory_,
    std::move(metadata_io_));

  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
  reader_->open(storage_options_, converter_options_);
}

TEST_F(SequentialCompressionReaderTest, compression_called_when_loading_split_bagfile)
{
  metadata_.topics_with_message_count.push_back({{topic_with_type_}, 10});
  metadata_.bag_size = 512000;

  auto decompressor = std::make_unique<NiceMock<MockDecompressor>>();
  // We are mocking two splits, so file decompression should occur twice
  EXPECT_CALL(*decompressor, decompress_uri(_)).Times(2)
  .WillOnce(Return(metadata_.relative_file_paths[0]))
  .WillOnce(Return(metadata_.relative_file_paths[1]));
  EXPECT_CALL(*decompressor, decompress_serialized_bag_message(_)).Times(0);

  auto compression_factory = std::make_unique<StrictMock<MockCompressionFactory>>();
  ON_CALL(*compression_factory, create_decompressor(_))
  .WillByDefault(Return(ByMove(std::move(decompressor))));
  EXPECT_CALL(*compression_factory, create_decompressor(_)).Times(1);
  // open_read_only should be called twice when opening 2 split bags
  EXPECT_CALL(*storage_factory_, open_read_only(_)).Times(2);
  EXPECT_CALL(*storage_, has_next()).Times(3)
  .WillOnce(Return(false))  // Load the next file
  .WillOnce(Return(true))
  .WillOnce(Return(true));

  auto compression_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(compression_factory),
    std::move(storage_factory_),
    converter_factory_,
    std::move(metadata_io_));

  compression_reader->open(storage_options_, converter_options_);
  EXPECT_EQ(compression_reader->has_next_file(), true);
  EXPECT_EQ(compression_reader->has_next(), true);  // false then true
  compression_reader->read_next();  // calls has_next true
}

TEST_F(SequentialCompressionReaderTest, can_find_v4_names)
{
  auto reader = create_reader();
  reader->open(storage_options_, converter_options_);
  EXPECT_TRUE(reader->has_next_file());
}

TEST_F(SequentialCompressionReaderTest, throws_on_incorrect_filenames)
{
  for (auto & relative_file_path : metadata_.relative_file_paths) {
    relative_file_path = (
      rcpputils::fs::path(bag_name_) / (relative_file_path + ".something")).string();
  }
  auto reader = create_reader();
  EXPECT_THROW(reader->open(storage_options_, converter_options_), std::invalid_argument);
}

TEST_F(SequentialCompressionReaderTest, can_find_prefixed_filenames)
{
  // By prefixing the bag name, this imitates the V3 filename logic
  for (auto & relative_file_path : metadata_.relative_file_paths) {
    relative_file_path = (rcpputils::fs::path(bag_name_) / relative_file_path).string();
  }
  auto reader = create_reader();

  EXPECT_NO_THROW(reader->open(storage_options_, converter_options_));
  EXPECT_TRUE(reader->has_next_file());
}

TEST_F(SequentialCompressionReaderTest, can_find_prefixed_filenames_in_renamed_bag)
{
  // By prefixing a _different_ path than the bagname, we imitate the situation where the bag
  // was recorded using V3 logic, then the directory was moved to be a new name - this is the
  // use case the V4 relative path logic was intended to fix
  for (auto & relative_file_path : metadata_.relative_file_paths) {
    relative_file_path = (rcpputils::fs::path("OtherBagName") / relative_file_path).string();
  }
  auto reader = create_reader();

  EXPECT_NO_THROW(reader->open(storage_options_, converter_options_));
  EXPECT_TRUE(reader->has_next_file());
}

TEST_F(SequentialCompressionReaderTest, does_not_decompress_again_on_seek)
{
  auto decompressor = std::make_unique<NiceMock<MockDecompressor>>();
  ON_CALL(*decompressor, decompress_uri(_)).WillByDefault(Return("some/path"));
  EXPECT_CALL(*decompressor, decompress_uri(_)).Times(1);

  auto compression_factory = std::make_unique<NiceMock<MockCompressionFactory>>();
  ON_CALL(*compression_factory, create_decompressor(_))
  .WillByDefault(Return(ByMove(std::move(decompressor))));

  ON_CALL(*storage_, has_next()).WillByDefault(Return(true));

  auto sequential_reader = std::make_unique<rosbag2_compression::SequentialCompressionReader>(
    std::move(compression_factory),
    std::move(storage_factory_),
    converter_factory_,
    std::move(metadata_io_));

  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
  reader_->open(storage_options_, converter_options_);
  reader_->seek(0);
}
