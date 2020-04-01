// Copyright 2018, Bosch Software Innovations GmbH.
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

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT

class SequentialWriterTest : public Test
{
public:
  SequentialWriterTest()
  {
    storage_factory_ = std::make_unique<StrictMock<MockStorageFactory>>();
    storage_ = std::make_shared<NiceMock<MockStorage>>();
    converter_factory_ = std::make_shared<StrictMock<MockConverterFactory>>();
    metadata_io_ = std::make_unique<NiceMock<MockMetadataIo>>();
    storage_options_ = rosbag2_cpp::StorageOptions{};
    storage_options_.uri = "uri";

    ON_CALL(*storage_factory_, open_read_write(_, _)).WillByDefault(
      DoAll(
        Invoke(
          [this](const std::string & uri, const std::string &) {
            fake_storage_size_ = 0;
            fake_storage_uri_ = uri;
          }),
        Return(storage_)));
    EXPECT_CALL(
      *storage_factory_, open_read_write(_, _)).Times(AtLeast(0));
  }

  std::unique_ptr<StrictMock<MockStorageFactory>> storage_factory_;
  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<MockMetadataIo> metadata_io_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rosbag2_cpp::StorageOptions storage_options_;
  uint64_t fake_storage_size_;
  rosbag2_storage::BagMetadata fake_metadata_;
  std::string fake_storage_uri_;
};

TEST_F(
  SequentialWriterTest,
  write_uses_converters_to_convert_serialization_format_if_input_and_output_format_are_different) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string storage_serialization_format = "rmw1_format";
  std::string input_format = "rmw2_format";

  auto format1_converter = std::make_unique<StrictMock<MockConverter>>();
  auto format2_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(*format1_converter, serialize(_, _, _)).Times(1);
  EXPECT_CALL(*format2_converter, deserialize(_, _, _)).Times(1);

  EXPECT_CALL(*converter_factory_, load_serializer(storage_serialization_format))
  .WillOnce(Return(ByMove(std::move(format1_converter))));
  EXPECT_CALL(*converter_factory_, load_deserializer(input_format))
  .WillOnce(Return(ByMove(std::move(format2_converter))));

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";
  writer_->open(storage_options_, {input_format, storage_serialization_format});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});
  writer_->write(message);
}

TEST_F(SequentialWriterTest, write_does_not_use_converters_if_input_and_output_format_are_equal) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string storage_serialization_format = "rmw_format";

  EXPECT_CALL(*converter_factory_, load_deserializer(storage_serialization_format)).Times(0);
  EXPECT_CALL(*converter_factory_, load_serializer(storage_serialization_format)).Times(0);

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";
  writer_->open(storage_options_, {storage_serialization_format, storage_serialization_format});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});
  writer_->write(message);
}

TEST_F(SequentialWriterTest, metadata_io_writes_metadata_file_in_destructor) {
  EXPECT_CALL(*metadata_io_, write_metadata(_, _)).Times(1);
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";

  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_.reset();
}

TEST_F(SequentialWriterTest, open_throws_error_if_converter_plugin_does_not_exist) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string input_format = "rmw1_format";
  std::string output_format = "rmw2_format";

  auto format1_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(*converter_factory_, load_deserializer(input_format))
  .WillOnce(Return(ByMove(std::move(format1_converter))));
  EXPECT_CALL(*converter_factory_, load_serializer(output_format))
  .WillOnce(Return(ByMove(nullptr)));

  EXPECT_ANY_THROW(writer_->open(storage_options_, {input_format, output_format}));
}

TEST_F(SequentialWriterTest, open_throws_error_on_invalid_splitting_size) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  // Set minimum file size greater than max bagfile size option
  const uint64_t min_split_file_size = 10;
  const uint64_t max_bagfile_size = 5;
  ON_CALL(*storage_, get_minimum_split_file_size()).WillByDefault(Return(min_split_file_size));
  storage_options_.max_bagfile_size = max_bagfile_size;

  EXPECT_CALL(*storage_, get_minimum_split_file_size).Times(1);

  std::string rmw_format = "rmw_format";

  EXPECT_ANY_THROW(writer_->open(storage_options_, {rmw_format, rmw_format}));
}

TEST_F(SequentialWriterTest, bagfile_size_is_checked_on_every_write) {
  const int counter = 10;
  const uint64_t max_bagfile_size = 100;

  EXPECT_CALL(*storage_, get_bagfile_size()).Times(counter);

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";

  storage_options_.max_bagfile_size = max_bagfile_size;

  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});

  for (auto i = 0; i < counter; ++i) {
    writer_->write(message);
  }
}

TEST_F(SequentialWriterTest, writer_splits_when_storage_bagfile_size_gt_max_bagfile_size) {
  const int message_count = 15;
  const int max_bagfile_size = 5;
  const auto expected_splits = message_count / max_bagfile_size;
  fake_storage_size_ = 0;

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

  EXPECT_CALL(*metadata_io_, write_metadata).Times(1);

  // intercept the metadata write so we can analyze it.
  ON_CALL(*metadata_io_, write_metadata).WillByDefault(
    [this](const std::string &, const rosbag2_storage::BagMetadata & metadata) {
      fake_metadata_ = metadata;
    });

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";

  storage_options_.max_bagfile_size = max_bagfile_size;

  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});

  for (auto i = 0; i < message_count; ++i) {
    writer_->write(message);
  }

  writer_.reset();
  // metadata should be written now that the Writer was released.

  EXPECT_EQ(
    fake_metadata_.relative_file_paths.size(),
    static_cast<unsigned int>(expected_splits)) <<
    "Storage should have split bagfile " << (expected_splits - 1);

  const auto base_path =
    (rcpputils::fs::path(storage_options_.uri) / storage_options_.uri).string();
  int counter = 0;
  for (const auto & path : fake_metadata_.relative_file_paths) {
    std::stringstream ss;
    ss << base_path << "_" << counter;

    const auto expected_path = ss.str();
    counter++;
    EXPECT_EQ(expected_path, path);
  }
}
