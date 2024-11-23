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

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/default_storage_id.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"

#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT
using rosbag2_test_common::ParametrizedTemporaryDirectoryFixture;

namespace fs = rcpputils::fs;

class SequentialWriterTest : public Test
{
public:
  SequentialWriterTest()
  {
    storage_factory_ = std::make_unique<StrictMock<MockStorageFactory>>();
    storage_ = std::make_shared<NiceMock<MockStorage>>();
    converter_factory_ = std::make_shared<StrictMock<MockConverterFactory>>();
    metadata_io_ = std::make_unique<NiceMock<MockMetadataIo>>();
    tmp_dir_ = rcpputils::fs::temp_directory_path() / "SequentialWriterTest";
    storage_options_ = rosbag2_storage::StorageOptions{};
    storage_options_.uri = (tmp_dir_ / bag_base_dir_).string();

    rcpputils::fs::remove_all(tmp_dir_);

    ON_CALL(*storage_factory_, open_read_write(_)).WillByDefault(
      DoAll(
        Invoke(
          [this](const rosbag2_storage::StorageOptions & storage_options) {
            fake_storage_size_ = 0;
            fake_storage_uri_ = storage_options.uri;
          }),
        Return(storage_)));
    EXPECT_CALL(
      *storage_factory_, open_read_write(_)).Times(AtLeast(0));
  }

  ~SequentialWriterTest()
  {
    rcpputils::fs::remove_all(tmp_dir_);
  }

  std::unique_ptr<StrictMock<MockStorageFactory>> storage_factory_;
  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<MockMetadataIo> metadata_io_;

  fs::path tmp_dir_;
  rosbag2_storage::StorageOptions storage_options_;
  std::atomic<uint32_t> fake_storage_size_{0};  // Need to be atomic for cache update since it
  // uses in callback from cache_consumer thread
  rosbag2_storage::BagMetadata fake_metadata_;
  //  Ensure writer_ is destructed before intercepted fake_metadata_
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string fake_storage_uri_;
  const std::string bag_base_dir_ = "test_bag";
};

std::shared_ptr<rosbag2_storage::SerializedBagMessage> make_test_msg()
{
  static uint32_t counter = 0;
  std::string msg_content = "Hello" + std::to_string(counter++);
  auto msg_length = msg_content.length();
  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";
  message->serialized_data = rosbag2_storage::make_serialized_message(
    msg_content.c_str(), msg_length);
  return message;
}

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

  EXPECT_CALL(*storage_, get_minimum_split_file_size).Times(2);

  std::string rmw_format = "rmw_format";

  EXPECT_THROW(writer_->open(storage_options_, {rmw_format, rmw_format}), std::runtime_error);
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
      fake_storage_size_++;
    });

  ON_CALL(*storage_, get_bagfile_size).WillByDefault(
    [this]() {
      return fake_storage_size_.load();
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

  int counter = 0;
  for (const auto & path : fake_metadata_.relative_file_paths) {
    std::stringstream ss;
    ss << bag_base_dir_ << "_" << counter;

    const auto expected_path = ss.str();
    counter++;
    EXPECT_EQ(expected_path, path);
  }
}

TEST_F(
  SequentialWriterTest,
  writer_with_cache_splits_when_storage_bagfile_size_gt_max_bagfile_size) {
  const size_t message_count = 15;
  const size_t expected_total_written_messages = message_count - 1;
  const size_t max_bagfile_size = 5;
  const auto expected_splits = message_count / max_bagfile_size;
  fake_storage_size_ = 0;
  size_t written_messages = 0;

  ON_CALL(
    *storage_,
    write(An<const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> &>())).
  WillByDefault(
    [this, &written_messages]
      (const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msgs)
    {
      written_messages += msgs.size();
      fake_storage_size_.fetch_add(static_cast<uint32_t>(msgs.size()));
    });

  ON_CALL(*storage_, get_bagfile_size).WillByDefault(
    [this]() {
      return fake_storage_size_.load();
    });

  ON_CALL(*storage_, get_relative_file_path).WillByDefault(
    [this]() {
      return fake_storage_uri_;
    });

  EXPECT_CALL(*metadata_io_, write_metadata).Times(1);

  EXPECT_CALL(*storage_factory_, open_read_write(_)).Times(3);

  // intercept the metadata write so we can analyze it.
  ON_CALL(*metadata_io_, write_metadata).WillByDefault(
    [this](const std::string &, const rosbag2_storage::BagMetadata & metadata) {
      fake_metadata_ = metadata;
    });

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";

  storage_options_.max_bagfile_size = max_bagfile_size;
  storage_options_.max_cache_size = 4000u;
  storage_options_.snapshot_mode = false;

  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});

  auto timeout = std::chrono::seconds(2);
  for (auto i = 1u; i < message_count; ++i) {
    writer_->write(make_test_msg());
    // Wait for written_messages == i for each 5th message with timeout in 2 sec
    // Need yield resources and make sure that cache_consumer had a chance to dump buffer to the
    // storage before split is gonna occur. i.e. each 5th message.
    if ((i % max_bagfile_size) == 0) {
      auto start_time = std::chrono::steady_clock::now();
      while ((i != written_messages) &&
        (std::chrono::steady_clock::now() - start_time < timeout))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      EXPECT_EQ(i, written_messages);
    }
    if ((i % max_bagfile_size) == 1) {  // Check on the 6th and 11 message that split happened.
      // i.e. fake_storage_size_ zeroed on split and then incremented in cache_consumer callback.
      auto start_time = std::chrono::steady_clock::now();
      while ((fake_storage_size_ != 1u) &&
        ((std::chrono::steady_clock::now() - start_time) < timeout))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      EXPECT_EQ(fake_storage_size_, 1u) << "current message number = " << i;
    }
  }

  writer_.reset();
  EXPECT_EQ(written_messages, expected_total_written_messages);

// metadata should be written now that the Writer was released.
  EXPECT_EQ(
    fake_metadata_.relative_file_paths.size(),
    static_cast<unsigned int>(expected_splits)) <<
    "Storage should have split bagfile " << (expected_splits - 1);

  int counter = 0;
  for (const auto & path : fake_metadata_.relative_file_paths) {
    std::stringstream ss;
    ss << bag_base_dir_ << "_" << counter;

    const auto expected_path = ss.str();
    counter++;
    EXPECT_EQ(expected_path, path);
  }
}

TEST_F(SequentialWriterTest, do_not_use_cache_if_cache_size_is_zero) {
  const size_t counter = 1000;
  const uint64_t max_cache_size = 0;

  EXPECT_CALL(
    *storage_,
    write(An<const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> &>())).
  Times(0);
  EXPECT_CALL(
    *storage_,
    write(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>())).Times(counter);

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";

  std::string msg_content = "Hello";
  auto msg_length = msg_content.length();
  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";
  message->serialized_data = rosbag2_storage::make_serialized_message(
    msg_content.c_str(), msg_length);


  storage_options_.max_bagfile_size = 0;
  storage_options_.max_cache_size = max_cache_size;

  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});

  for (auto i = 0u; i < counter; ++i) {
    writer_->write(message);
  }
}

TEST_F(SequentialWriterTest, snapshot_mode_write_on_trigger)
{
  storage_options_.max_bagfile_size = 0;
  storage_options_.max_cache_size = 200;
  storage_options_.snapshot_mode = true;

  // Expect a single write call when the snapshot is triggered
  EXPECT_CALL(
    *storage_, write(
      An
      <const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> &>())
  ).Times(1);

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";

  std::string msg_content = "Hello";
  auto msg_length = msg_content.length();
  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";
  message->serialized_data = rosbag2_storage::make_serialized_message(
    msg_content.c_str(), msg_length);

  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});

  for (auto i = 0u; i < 100; ++i) {
    writer_->write(message);
  }
  writer_->take_snapshot();
}

TEST_F(SequentialWriterTest, snapshot_mode_not_triggered_no_storage_write)
{
  storage_options_.max_bagfile_size = 0;
  storage_options_.max_cache_size = 200;
  storage_options_.snapshot_mode = true;

  // Storage should never be written to when snapshot mode is enabled
  // but a snapshot is never triggered
  EXPECT_CALL(
    *storage_, write(
      An
      <const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> &>())
  ).Times(0);

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";

  std::string msg_content = "Hello";
  auto msg_length = msg_content.length();
  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";
  message->serialized_data = rosbag2_storage::make_serialized_message(
    msg_content.c_str(), msg_length);

  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});

  for (auto i = 0u; i < 100; ++i) {
    writer_->write(message);
  }
}

TEST_F(SequentialWriterTest, snapshot_mode_zero_cache_size_throws_exception)
{
  storage_options_.max_bagfile_size = 0;
  storage_options_.max_cache_size = 0;
  storage_options_.snapshot_mode = true;

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";
  EXPECT_THROW(writer_->open(storage_options_, {rmw_format, rmw_format}), std::runtime_error);
}

TEST_F(SequentialWriterTest, snapshot_writes_to_new_file_with_bag_split)
{
  storage_options_.max_bagfile_size = 0;
  storage_options_.max_cache_size = 200;
  storage_options_.snapshot_mode = true;
  const rcutils_time_point_value_t first_msg_timestamp = 100;
  const size_t num_msgs_to_write = 100;
  const std::string topic_name = "test_topic";
  std::string msg_content = "Hello";
  const size_t serialized_msg_buffer_length = msg_content.length();
  const size_t num_expected_msgs = storage_options_.max_cache_size / serialized_msg_buffer_length;
  const size_t expected_start_time = first_msg_timestamp + (num_msgs_to_write - num_expected_msgs);
  const auto expected_last_msg_timestamp = (first_msg_timestamp + num_msgs_to_write - 1);
  const size_t expected_duration = expected_last_msg_timestamp - expected_start_time;
  // Prepare vector of messages
  std::vector<rosbag2_storage::SerializedBagMessageSharedPtr> messages;
  for (size_t i = 0; i < num_msgs_to_write; i++) {
    auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    message->time_stamp = first_msg_timestamp + static_cast<rcutils_time_point_value_t>(i);
    message->topic_name = topic_name;
    message->serialized_data =
      rosbag2_storage::make_serialized_message(msg_content.c_str(), msg_content.length());
    messages.push_back(message);
  }

  // Expect a single write call when the snapshot is triggered
  EXPECT_CALL(
    *storage_, write(
      An<const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> &>())
  ).Times(1);

  ON_CALL(
    *storage_,
    write(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>())).WillByDefault(
    [this](std::shared_ptr<const rosbag2_storage::SerializedBagMessage>) {
      fake_storage_size_ += 1;
    });

  ON_CALL(*storage_, get_bagfile_size).WillByDefault(
    [this]() {
      return fake_storage_size_.load();
    });

  ON_CALL(*metadata_io_, write_metadata).WillByDefault(
    [this](const std::string &, const rosbag2_storage::BagMetadata & metadata) {
      fake_metadata_ = metadata;
    });

  ON_CALL(*storage_, get_relative_file_path).WillByDefault(
    [this]() {
      return fake_storage_uri_;
    });

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::vector<std::string> closed_files;
  std::vector<std::string> opened_files;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&closed_files, &opened_files](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      closed_files.emplace_back(info.closed_file);
      opened_files.emplace_back(info.opened_file);
    };
  writer_->add_event_callbacks(callbacks);

  std::string rmw_format = "rmw_format";

  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});

  for (const auto & message : messages) {
    writer_->write(message);
  }
  writer_->take_snapshot();
  writer_->close();

  EXPECT_THAT(closed_files.size(), 2);
  EXPECT_THAT(opened_files.size(), 2);

  if (!((closed_files.size() == opened_files.size()) && (opened_files.size() == 2))) {
    // Output debug info
    for (size_t i = 0; i < opened_files.size(); i++) {
      std::cout << "opened_file[" << i << "] = '" << opened_files[i] <<
        "'; closed_file[" << i << "] = '" << closed_files[i] << "';" << std::endl;
    }
  }

  ASSERT_EQ(opened_files.size(), 2);
  ASSERT_EQ(closed_files.size(), 2);

  for (size_t i = 0; i < 2; i++) {
    auto expected_closed =
      fs::path(storage_options_.uri) / (bag_base_dir_ + "_" + std::to_string(i));
    auto expected_opened = (i == 1) ?
      // The last opened file shall be empty string when we do "writer->close();"
      fs::path("") : fs::path(storage_options_.uri) / (bag_base_dir_ + "_" + std::to_string(i + 1));
    ASSERT_STREQ(closed_files[i].c_str(), expected_closed.string().c_str());
    ASSERT_STREQ(opened_files[i].c_str(), expected_opened.string().c_str());
  }
  // Check metadata
  EXPECT_EQ(fake_metadata_.message_count, num_expected_msgs);
  EXPECT_EQ(
    std::chrono::time_point_cast<std::chrono::nanoseconds>(
      fake_metadata_.starting_time).time_since_epoch().count(), first_msg_timestamp);

  ASSERT_FALSE(fake_metadata_.files.empty());

  const auto & first_file_info = fake_metadata_.files[0];
  EXPECT_STREQ(first_file_info.path.c_str(), std::string(bag_base_dir_ + "_0").c_str());
  EXPECT_EQ(first_file_info.message_count, num_expected_msgs);
  EXPECT_EQ(
    std::chrono::time_point_cast<std::chrono::nanoseconds>(
      first_file_info.starting_time).time_since_epoch().count(), expected_start_time);
  EXPECT_EQ(first_file_info.duration.count(), expected_duration);
}

TEST_F(SequentialWriterTest, snapshot_can_be_called_twice)
{
  storage_options_.max_bagfile_size = 0;
  storage_options_.max_cache_size = 200;
  storage_options_.snapshot_mode = true;
  const size_t num_msgs_to_write = 100;

  // Expect to call write method twice. Once per each snapshot.
  EXPECT_CALL(
    *storage_, write(
      An<const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> &>())
  ).Times(2);

  ON_CALL(*storage_, get_relative_file_path).WillByDefault(
    [this]() {
      return fake_storage_uri_;
    });

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::vector<std::string> closed_files;
  std::vector<std::string> opened_files;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&closed_files, &opened_files](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      closed_files.emplace_back(info.closed_file);
      opened_files.emplace_back(info.opened_file);
    };
  writer_->add_event_callbacks(callbacks);

  std::string rmw_format = "rmw_format";

  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});

  std::string msg_content = "Hello";
  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";
  message->serialized_data =
    rosbag2_storage::make_serialized_message(msg_content.c_str(), msg_content.length());

  for (size_t i = 0; i < num_msgs_to_write / 2; i++) {
    writer_->write(message);
  }
  writer_->take_snapshot();

  for (size_t i = num_msgs_to_write / 2; i < num_msgs_to_write; i++) {
    writer_->write(message);
  }
  writer_->take_snapshot();

  EXPECT_THAT(closed_files.size(), 2);
  EXPECT_THAT(opened_files.size(), 2);

  if (!((closed_files.size() == opened_files.size()) && (opened_files.size() == 2))) {
    // Output debug info
    for (size_t i = 0; i < opened_files.size(); i++) {
      std::cout << "opened_file[" << i << "] = '" << opened_files[i] <<
        "'; closed_file[" << i << "] = '" << closed_files[i] << "';" << std::endl;
    }
  }

  ASSERT_EQ(opened_files.size(), 2);
  ASSERT_EQ(closed_files.size(), 2);

  for (size_t i = 0; i < opened_files.size(); i++) {
    const auto expected_closed = fs::path(storage_options_.uri) /
      (bag_base_dir_ + "_" + std::to_string(i));
    const auto expected_opened = fs::path(storage_options_.uri) /
      (bag_base_dir_ + "_" + std::to_string(i + 1));
    ASSERT_STREQ(closed_files[i].c_str(), expected_closed.string().c_str());
    ASSERT_STREQ(opened_files[i].c_str(), expected_opened.string().c_str());
  }
}

TEST_F(SequentialWriterTest, split_event_calls_callback)
{
  const uint64_t max_bagfile_size = 3;
  const size_t num_splits = 2;
  const int message_count = max_bagfile_size * num_splits + max_bagfile_size - 1;  // 8

  ON_CALL(
    *storage_,
    write(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>())).WillByDefault(
    [this](std::shared_ptr<const rosbag2_storage::SerializedBagMessage>) {
      fake_storage_size_ += 1;
    });

  ON_CALL(*storage_, get_bagfile_size).WillByDefault(
    [this]() {
      return fake_storage_size_.load();
    });

  ON_CALL(*metadata_io_, write_metadata).WillByDefault(
    [this](const std::string &, const rosbag2_storage::BagMetadata & metadata) {
      fake_metadata_ = metadata;
    });

  ON_CALL(*storage_, get_relative_file_path).WillByDefault(
    [this]() {
      return fake_storage_uri_;
    });

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";

  storage_options_.max_bagfile_size = max_bagfile_size;

  std::vector<std::string> closed_files;
  std::vector<std::string> opened_files;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&closed_files, &opened_files](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      closed_files.emplace_back(info.closed_file);
      opened_files.emplace_back(info.opened_file);
    };
  writer_->add_event_callbacks(callbacks);

  writer_->open(storage_options_, {"rmw_format", "rmw_format"});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});

  for (auto i = 0; i < message_count; ++i) {
    writer_->write(message);
  }
  writer_->close();

  EXPECT_THAT(closed_files.size(), num_splits + 1);
  EXPECT_THAT(opened_files.size(), num_splits + 1);

  if (!((closed_files.size() == opened_files.size()) && (opened_files.size() == num_splits + 1))) {
    // Output debug info
    for (size_t i = 0; i < opened_files.size(); i++) {
      std::cout << "opened_file[" << i << "] = '" << opened_files[i] <<
        "'; closed_file[" << i << "] = '" << closed_files[i] << "';" << std::endl;
    }
  }

  ASSERT_GE(opened_files.size(), num_splits + 1);
  ASSERT_GE(closed_files.size(), num_splits + 1);
  for (size_t i = 0; i < num_splits + 1; i++) {
    auto expected_closed =
      fs::path(storage_options_.uri) / (bag_base_dir_ + "_" + std::to_string(i));
    auto expected_opened = (i == num_splits) ?
      // The last opened file shall be empty string when we do "writer->close();"
      fs::path("") : fs::path(storage_options_.uri) / (bag_base_dir_ + "_" + std::to_string(i + 1));
    EXPECT_EQ(closed_files[i], expected_closed.string());
    EXPECT_EQ(opened_files[i], expected_opened.string());
  }
}

TEST_F(SequentialWriterTest, split_event_calls_on_writer_close)
{
  const int message_count = 7;

  ON_CALL(
    *storage_,
    write(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>())).WillByDefault(
    [this](std::shared_ptr<const rosbag2_storage::SerializedBagMessage>) {
      fake_storage_size_ += 1;
    });

  ON_CALL(*storage_, get_bagfile_size).WillByDefault(
    [this]() {
      return fake_storage_size_.load();
    });

  ON_CALL(*metadata_io_, write_metadata).WillByDefault(
    [this](const std::string &, const rosbag2_storage::BagMetadata & metadata) {
      fake_metadata_ = metadata;
    });

  ON_CALL(*storage_, get_relative_file_path).WillByDefault(
    [this]() {
      return fake_storage_uri_;
    });

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";

  storage_options_.max_bagfile_size = 0;

  bool callback_called = false;
  std::string closed_file, opened_file;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&callback_called, &closed_file, &opened_file](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      closed_file = info.closed_file;
      opened_file = info.opened_file;
      callback_called = true;
    };
  writer_->add_event_callbacks(callbacks);

  writer_->open(storage_options_, {"rmw_format", "rmw_format"});
  writer_->create_topic({"test_topic", "test_msgs/BasicTypes", "", ""});

  for (auto i = 0; i < message_count; ++i) {
    writer_->write(message);
  }
  writer_->close();

  ASSERT_TRUE(callback_called);
  auto expected_closed = rcpputils::fs::path(storage_options_.uri) / (bag_base_dir_ + "_0");
  EXPECT_EQ(closed_file, expected_closed.string());
  EXPECT_TRUE(opened_file.empty());
}

class ManualSplitWriter : public rosbag2_cpp::writers::SequentialWriter
{
public:
  // makes the method public for manual splitting
  void split()
  {
    split_bagfile();
  }
};

void write_sample_split_bag(
  const rosbag2_storage::StorageOptions & storage_options,
  const std::vector<std::vector<rcutils_time_point_value_t>> & message_timestamps_by_file)
{
  std::string msg_content = "Hello";
  auto msg_length = msg_content.length();
  std::shared_ptr<rcutils_uint8_array_t> fake_data = rosbag2_storage::make_serialized_message(
    msg_content.c_str(), msg_length);
  std::string topic_name = "testtopic";

  ManualSplitWriter writer;
  writer.open(storage_options, rosbag2_cpp::ConverterOptions{});
  writer.create_topic(
  {
    topic_name,
    "test_msgs/ByteMultiArray",
    "cdr",
    ""
  });
  for (const auto & file_messages : message_timestamps_by_file) {
    for (const auto time_stamp : file_messages) {
      auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      msg->serialized_data = fake_data;
      msg->time_stamp = time_stamp;
      msg->topic_name = topic_name;
      writer.write(msg);
    }
    writer.split();
  }
  writer.close();
}

TEST_P(ParametrizedTemporaryDirectoryFixture, split_bag_metadata_has_full_duration) {
  const std::vector<std::vector<rcutils_time_point_value_t>> message_timestamps_by_file {
    {100, 300, 200},
    {500, 400, 600}
  };
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = (rcpputils::fs::path(temporary_dir_path_) / "split_duration_bag").string();
  storage_options.storage_id = GetParam();
  write_sample_split_bag(storage_options, message_timestamps_by_file);

  rosbag2_storage::MetadataIo metadata_io;
  auto metadata = metadata_io.read_metadata(storage_options.uri);
  ASSERT_EQ(
    metadata.starting_time,
    std::chrono::high_resolution_clock::time_point(std::chrono::nanoseconds(100)));
  ASSERT_EQ(metadata.duration, std::chrono::nanoseconds(500));
}

INSTANTIATE_TEST_SUITE_P(
  SplitMetadataTest,
  ParametrizedTemporaryDirectoryFixture,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
