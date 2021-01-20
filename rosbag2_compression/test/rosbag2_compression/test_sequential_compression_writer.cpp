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

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"

#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_storage/storage_options.hpp"

#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

#include "mock_compression_factory.hpp"

using namespace testing;  // NOLINT

class SequentialCompressionWriterTest : public Test
{
public:
  SequentialCompressionWriterTest()
  : storage_factory_{std::make_unique<StrictMock<MockStorageFactory>>()},
    storage_{std::make_shared<NiceMock<MockStorage>>()},
    converter_factory_{std::make_shared<StrictMock<MockConverterFactory>>()},
    metadata_io_{std::make_unique<NiceMock<MockMetadataIo>>()},
    tmp_dir_{rcpputils::fs::temp_directory_path() / bag_name_},
    tmp_dir_storage_options_{},
    serialization_format_{"rmw_format"}
  {
    tmp_dir_storage_options_.uri = tmp_dir_.string();
    rcpputils::fs::remove_all(tmp_dir_);
    ON_CALL(*storage_factory_, open_read_write(_)).WillByDefault(Return(storage_));
    EXPECT_CALL(*storage_factory_, open_read_write(_)).Times(AtLeast(0));
    // intercept the metadata write so we can analyze it.
    ON_CALL(*metadata_io_, write_metadata).WillByDefault(
      [this](const std::string &, const rosbag2_storage::BagMetadata & metadata) {
        intercepted_metadata_ = metadata;
      });
  }

  void initializeFakeFileStorage()
  {
    // Create mock implementation of the storage, using files and a size of 1 per message
    // initialize values when opening a new bagfile
    ON_CALL(*storage_factory_, open_read_write(_)).WillByDefault(
      DoAll(
        Invoke(
          [this](const rosbag2_storage::StorageOptions & storage_options) {
            fake_storage_size_ = 0;
            fake_storage_uri_ = storage_options.uri;
            // Touch the file
            std::ofstream output(storage_options.uri);
            // Put some arbitrary bytes in the file so it isn't interpreted as being empty
            output << "Fake storage data";
          }),
        Return(storage_)));
    ON_CALL(
      *storage_,
      write(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>())).WillByDefault(
      [this](std::shared_ptr<const rosbag2_storage::SerializedBagMessage>) {
        fake_storage_size_ += 1;
      });
    ON_CALL(*storage_, get_bagfile_size).WillByDefault(
      [this]() {
        return fake_storage_size_;
      });
    ON_CALL(*storage_, get_relative_file_path).WillByDefault(
      [this]() {
        return fake_storage_uri_;
      });
  }

  void initializeWriter(
    const rosbag2_compression::CompressionOptions & compression_options,
    std::unique_ptr<rosbag2_compression::CompressionFactory> custom_compression_factory = nullptr)
  {
    auto compression_factory = std::move(custom_compression_factory);
    if (!compression_factory) {
      compression_factory = std::make_unique<rosbag2_compression::CompressionFactory>();
    }
    auto sequential_writer = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(
      compression_options,
      std::move(compression_factory),
      std::move(storage_factory_),
      converter_factory_,
      std::move(metadata_io_));
    writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));
  }

  const std::string bag_name_ = "SequentialCompressionWriterTest";
  std::unique_ptr<StrictMock<MockStorageFactory>> storage_factory_;
  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<MockMetadataIo> metadata_io_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rcpputils::fs::path tmp_dir_;
  rosbag2_storage::StorageOptions tmp_dir_storage_options_;
  rosbag2_storage::BagMetadata intercepted_metadata_;
  std::string serialization_format_;
  uint64_t fake_storage_size_;
  std::string fake_storage_uri_;

  const uint64_t kDefaultCompressionQueueSize = 1;
  const uint64_t kDefaultCompressionQueueThreads = 4;
};

TEST_F(SequentialCompressionWriterTest, open_throws_on_empty_storage_options_uri)
{
  rosbag2_compression::CompressionOptions compression_options{
    "zstd", rosbag2_compression::CompressionMode::FILE,
    kDefaultCompressionQueueSize, kDefaultCompressionQueueThreads};
  initializeWriter(compression_options);

  EXPECT_THROW(
    writer_->open(
      rosbag2_storage::StorageOptions(),
      {serialization_format_, serialization_format_}),
    std::runtime_error);
}

TEST_F(SequentialCompressionWriterTest, open_throws_on_bad_compression_format)
{
  rosbag2_compression::CompressionOptions compression_options{
    "bad_format", rosbag2_compression::CompressionMode::FILE,
    kDefaultCompressionQueueSize, kDefaultCompressionQueueThreads};
  initializeWriter(compression_options);

  EXPECT_THROW(
    writer_->open(tmp_dir_storage_options_, {serialization_format_, serialization_format_}),
    std::invalid_argument);

  EXPECT_TRUE(rcpputils::fs::remove(tmp_dir_));
}

TEST_F(SequentialCompressionWriterTest, open_throws_on_invalid_splitting_size)
{
  // Set minimum file size greater than max bagfile size option
  const uint64_t min_split_file_size = 10;
  const uint64_t max_bagfile_size = 5;
  ON_CALL(*storage_, get_minimum_split_file_size()).WillByDefault(Return(min_split_file_size));

  rosbag2_compression::CompressionOptions compression_options{
    "zstd", rosbag2_compression::CompressionMode::FILE,
    kDefaultCompressionQueueSize, kDefaultCompressionQueueThreads};
  initializeWriter(compression_options);

  rosbag2_storage::StorageOptions storage_options{};
  storage_options.max_bagfile_size = max_bagfile_size;
  storage_options.uri = "foo.bar";
  EXPECT_THROW(
    writer_->open(storage_options, {serialization_format_, serialization_format_}),
    std::runtime_error);
}

TEST_F(SequentialCompressionWriterTest, open_succeeds_on_supported_compression_format)
{
  rosbag2_compression::CompressionOptions compression_options{
    "zstd", rosbag2_compression::CompressionMode::FILE,
    kDefaultCompressionQueueSize, kDefaultCompressionQueueThreads};
  initializeWriter(compression_options);

  auto tmp_dir = rcpputils::fs::temp_directory_path() / "path_not_empty";
  auto storage_options = rosbag2_storage::StorageOptions();
  storage_options.uri = tmp_dir.string();

  EXPECT_NO_THROW(
    writer_->open(tmp_dir_storage_options_, {serialization_format_, serialization_format_}));

  EXPECT_TRUE(rcpputils::fs::remove(tmp_dir_));
}

TEST_F(SequentialCompressionWriterTest, writer_calls_create_compressor)
{
  rosbag2_compression::CompressionOptions compression_options{
    "zstd", rosbag2_compression::CompressionMode::FILE,
    kDefaultCompressionQueueSize, kDefaultCompressionQueueThreads};
  auto compression_factory = std::make_unique<StrictMock<MockCompressionFactory>>();
  EXPECT_CALL(*compression_factory, create_compressor(_)).Times(1);

  initializeWriter(compression_options, std::move(compression_factory));

  // This will throw an exception because the MockCompressionFactory does not actually create
  // a compressor.
  EXPECT_THROW(
    writer_->open(tmp_dir_storage_options_, {serialization_format_, serialization_format_}),
    std::runtime_error);

  EXPECT_TRUE(rcpputils::fs::remove(tmp_dir_));
}

TEST_F(SequentialCompressionWriterTest, writer_creates_correct_metadata_relative_filepaths)
{
  // In this test, check that the SequentialCompressionWriter creates relative filepaths correctly
  // Check both the first path, which is created in init_metadata,
  // and subsequent paths, which are created in the splitting logic
  const std::string test_topic_name = "test_topic";
  const std::string test_topic_type = "test_msgs/BasicTypes";
  rosbag2_compression::CompressionOptions compression_options {
    "zstd",
    rosbag2_compression::CompressionMode::FILE,
    kDefaultCompressionQueueSize,
    kDefaultCompressionQueueThreads
  };

  initializeFakeFileStorage();
  initializeWriter(compression_options);

  tmp_dir_storage_options_.max_bagfile_size = 1;
  writer_->open(tmp_dir_storage_options_);
  writer_->create_topic({test_topic_name, test_topic_type, "", ""});

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = test_topic_name;

  writer_->write(message);
  // bag size == max_bafile_size, no split yet
  writer_->write(message);
  // bag size > max_bagfile_size, split
  writer_->write(message);
  writer_.reset();

  EXPECT_EQ(
    intercepted_metadata_.relative_file_paths.size(), 2u);

  const auto base_path = tmp_dir_storage_options_.uri;
  int counter = 0;
  for (const auto & path : intercepted_metadata_.relative_file_paths) {
    std::stringstream ss;
    ss << bag_name_ << "_" << counter << ".zstd";
    counter++;
    EXPECT_EQ(ss.str(), path);
  }
}
