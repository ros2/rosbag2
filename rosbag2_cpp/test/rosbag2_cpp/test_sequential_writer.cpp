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
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag2_test_common/temporary_directory_fixture.hpp"

#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT
using rosbag2_test_common::TemporaryDirectoryFixture;

class SequentialWriterTest : public Test
{
public:
  SequentialWriterTest()
  {
    storage_factory_ = std::make_unique<StrictMock<MockStorageFactory>>();
    storage_ = std::make_shared<NiceMock<MockStorage>>();
    converter_factory_ = std::make_shared<StrictMock<MockConverterFactory>>();
    metadata_io_ = std::make_unique<NiceMock<MockMetadataIo>>();
    storage_options_ = rosbag2_storage::StorageOptions{};
    storage_options_.uri = "uri";

    rcpputils::fs::path dir(storage_options_.uri);
    rcpputils::fs::remove_all(dir);

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
    ON_CALL(*storage_, get_relative_file_path).WillByDefault(
      [this]() {
        return fake_storage_uri_ + file_ext_;
      });
  }

  ~SequentialWriterTest()
  {
    rcpputils::fs::path dir(storage_options_.uri);
    rcpputils::fs::remove_all(dir);
  }

  std::unique_ptr<StrictMock<MockStorageFactory>> storage_factory_;
  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<MockMetadataIo> metadata_io_;

  rosbag2_storage::StorageOptions storage_options_;
  std::atomic<uint32_t> fake_storage_size_{0};  // Need to be atomic for cache update since it
  // uses in callback from cache_consumer thread
  rosbag2_storage::BagMetadata fake_metadata_;
  //  Ensure writer_ is destructed before intercepted fake_metadata_
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string fake_storage_uri_;
  const std::string file_ext_ = ".mock";
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

  const auto base_path = storage_options_.uri;
  int counter = 0;
  for (const auto & path : fake_metadata_.relative_file_paths) {
    std::stringstream ss;
    ss << base_path << "_" << counter << file_ext_;

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

  const auto base_path = storage_options_.uri;
  int counter = 0;
  for (const auto & path : fake_metadata_.relative_file_paths) {
    std::stringstream ss;
    ss << base_path << "_" << counter << file_ext_;

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

TEST_F(SequentialWriterTest, split_event_calls_callback)
{
  const int message_count = 7;
  const int max_bagfile_size = 5;

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

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";

  storage_options_.max_bagfile_size = max_bagfile_size;

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

  ASSERT_TRUE(callback_called);
  auto expected_closed = rcpputils::fs::path(storage_options_.uri) /
    (storage_options_.uri + "_0" + file_ext_);
  EXPECT_EQ(closed_file, expected_closed.string());
  EXPECT_EQ(opened_file, storage_->get_relative_file_path());
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
  auto expected_closed = rcpputils::fs::path(storage_options_.uri) / (storage_options_.uri + "_0");
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
  const std::string & uri,
  const std::vector<std::vector<rcutils_time_point_value_t>> & message_timestamps_by_file,
  const int max_bag_splits = 0)
{
  std::string msg_content = "Hello";
  auto msg_length = msg_content.length();
  std::shared_ptr<rcutils_uint8_array_t> fake_data = rosbag2_storage::make_serialized_message(
    msg_content.c_str(), msg_length);
  std::string topic_name = "testtopic";

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = uri;
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_splits = max_bag_splits;

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


TEST_F(TemporaryDirectoryFixture, split_bag_metadata_has_full_duration) {
  const std::vector<std::vector<rcutils_time_point_value_t>> message_timestamps_by_file {
    {100, 300, 200},
    {500, 400, 600}
  };
  std::string uri = (rcpputils::fs::path(temporary_dir_path_) / "split_duration_bag").string();
  write_sample_split_bag(uri, message_timestamps_by_file);

  rosbag2_storage::MetadataIo metadata_io;
  auto metadata = metadata_io.read_metadata(uri);
  ASSERT_EQ(
    metadata.starting_time,
    std::chrono::high_resolution_clock::time_point(std::chrono::nanoseconds(100)));
  ASSERT_EQ(metadata.duration, std::chrono::nanoseconds(500));
}

TEST_F(TemporaryDirectoryFixture, test_max_splits) {
  const std::vector<std::vector<rcutils_time_point_value_t>> message_timestamps_by_file {
    {100, 300, 200},
    {500, 400, 600},
    {700, 800, 900}
  };
  std::string uri = (rcpputils::fs::path(temporary_dir_path_) / "split_duration_bag").string();
  size_t max_bagfile_splits = 3;
  write_sample_split_bag(uri, message_timestamps_by_file, max_bagfile_splits);

  rosbag2_storage::MetadataIo metadata_io;
  auto metadata = metadata_io.read_metadata(uri);
  ASSERT_EQ(
    metadata.starting_time,
    std::chrono::high_resolution_clock::time_point(std::chrono::nanoseconds(400)));
  ASSERT_EQ(metadata.duration, std::chrono::nanoseconds(500));
  ASSERT_EQ(
    metadata.files.size(),
    max_bagfile_splits
  );
  ASSERT_EQ(
    metadata.duration, std::chrono::nanoseconds(500)
  );
}
TEST_F(SequentialWriterTest, open_throws_error_on_max_bagfile_splits_too_large) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  // Set max_bagfile_splits greater that rosbag2_storage::max_allowed_file_splits.
  storage_options_.max_bagfile_splits = 2147483648;
  std::string rmw_format = "rmw_format";

  EXPECT_THROW(writer_->open(storage_options_, {rmw_format, rmw_format}), std::runtime_error);
}

TEST_F(SequentialWriterTest, suffix_style_index)
{
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  storage_options_.suffix_style = rosbag2_storage::FileSuffixStyle::Index;
  writer_->open(storage_options_, {"", ""});

  auto expected = rcpputils::fs::path(storage_options_.uri) / (storage_options_.uri + "_0");
  EXPECT_EQ(fake_storage_uri_, expected.string());
  writer_->split_bagfile();
  expected = rcpputils::fs::path(storage_options_.uri) / (storage_options_.uri + "_1");
  EXPECT_EQ(fake_storage_uri_, expected.string());
}

TEST_F(SequentialWriterTest, suffix_style_datetime)
{
  std::string datetime_regex{"^uri/uri_[0-9]{8}T[0-9]{6}Z(\\.[0-9]+)?$"};
  auto no_ext = [](const std::string & path) -> std::string {
      return std::filesystem::path(path).replace_extension("").string();
    };

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  storage_options_.suffix_style = rosbag2_storage::FileSuffixStyle::Datetime;
  writer_->open(storage_options_, {"", ""});

  auto first_stem = no_ext(storage_->get_relative_file_path());
  EXPECT_THAT(first_stem, MatchesRegex(datetime_regex));

  writer_->split_bagfile();
  auto second_stem = no_ext(storage_->get_relative_file_path());
  EXPECT_THAT(second_stem, MatchesRegex(datetime_regex));
  EXPECT_THAT(second_stem, Gt(first_stem));

  writer_->split_bagfile();
  auto third_stem = no_ext(storage_->get_relative_file_path());
  EXPECT_THAT(third_stem, MatchesRegex(datetime_regex));
  EXPECT_THAT(third_stem, Gt(first_stem));
  EXPECT_THAT(third_stem, Gt(second_stem));
}

class SequentialWriterHelper : public rosbag2_cpp::writers::SequentialWriter
{
public:
  explicit SequentialWriterHelper(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
    std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface> converter_factory,
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
  : rosbag2_cpp::writers::SequentialWriter(
      std::move(storage_factory), converter_factory, std::move(metadata_io))
  {}

  rosbag2_storage::BagMetadata get_metadata()
  {
    return metadata_;
  }
};

class AppendWriteTest : public rosbag2_test_common::TemporaryDirectoryFixture
{
public:
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  rosbag2_storage::StorageOptions storage_options_;

  AppendWriteTest()
  {
    converter_factory_ = std::make_shared<StrictMock<MockConverterFactory>>();
    storage_options_.uri = (std::filesystem::path(temporary_dir_path_) / "uri").string();
  }
  virtual ~AppendWriteTest() = default;

  void basic_append_check()
  {
    auto sequential_writer = create_sequential_writer();
    auto writer_ptr = sequential_writer.get();
    auto writer = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));
    storage_options_.append_files = true;
    writer->open(storage_options_, {"", ""});
    writer->split_bagfile();
    writer->split_bagfile();
    size_t expected_splits = 3;
    size_t max_splits = storage_options_.max_bagfile_splits;
    size_t first_index_offset = 0;
    if (max_splits > 0 && max_splits < expected_splits) {
      first_index_offset = expected_splits - max_splits;
      expected_splits = max_splits;
    }

    auto meta = writer_ptr->get_metadata();
    EXPECT_THAT(meta.relative_file_paths, SizeIs(expected_splits));
    if (storage_options_.suffix_style == rosbag2_storage::FileSuffixStyle::Index) {
      for (size_t i = 0; i < expected_splits; i++) {
        EXPECT_EQ(
          std::filesystem::path(meta.relative_file_paths[i]).replace_extension("").string(),
          "uri_" + std::to_string(i + first_index_offset));
      }
    }
  }

  std::unique_ptr<SequentialWriterHelper> create_sequential_writer()
  {
    auto storage_factory = std::make_unique<rosbag2_storage::StorageFactory>();
    auto metadata_io = std::make_unique<rosbag2_storage::MetadataIo>();
    return std::make_unique<SequentialWriterHelper>(
      std::move(storage_factory), converter_factory_, std::move(metadata_io));
  }

  static std::string no_ext(const std::string & path)
  {
    return std::filesystem::path(path).replace_extension("").string();
  }
};


TEST_F(AppendWriteTest, append_nonexistent)
{
  storage_options_.max_bagfile_splits = 0;
  basic_append_check();
}

TEST_F(AppendWriteTest, append_index_on_existing_index)
{
  storage_options_.max_bagfile_splits = 2;
  basic_append_check();

  // Open new writer in append mode
  storage_options_.append_files = true;
  auto sequential_writer = create_sequential_writer();
  auto writer_ptr = sequential_writer.get();
  auto writer = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));
  writer->open(storage_options_, {"", ""});

  writer->split_bagfile();

  auto meta = writer_ptr->get_metadata();
  EXPECT_THAT(meta.relative_file_paths, SizeIs(2));
  EXPECT_EQ(no_ext(meta.relative_file_paths[0]), "uri_3");
  EXPECT_EQ(no_ext(meta.relative_file_paths[1]), "uri_4");
}

TEST_F(AppendWriteTest, append_index_on_existing_datetime)
{
  storage_options_.suffix_style = rosbag2_storage::FileSuffixStyle::Datetime;
  basic_append_check();

  // Close writer, open new one in append mode with index suffixing
  storage_options_.append_files = true;
  storage_options_.suffix_style = rosbag2_storage::FileSuffixStyle::Index;

  auto sequential_writer = create_sequential_writer();
  auto writer_ptr = sequential_writer.get();
  auto writer = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));
  writer->open(storage_options_, {"", ""});

  writer->split_bagfile();

  auto meta = writer_ptr->get_metadata();
  EXPECT_THAT(meta.relative_file_paths, SizeIs(5));
  EXPECT_EQ(no_ext(meta.relative_file_paths[3]), "uri_0");
  EXPECT_EQ(no_ext(meta.relative_file_paths[4]), "uri_1");
}

TEST_F(AppendWriteTest, append_index_on_existing_with_missing_metadata)
{
  storage_options_.suffix_style = rosbag2_storage::FileSuffixStyle::Index;
  storage_options_.max_bagfile_splits = 3;
  basic_append_check();
  auto metadata_path = std::filesystem::path(storage_options_.uri) / "metadata.yaml";
  ASSERT_TRUE(std::filesystem::remove(metadata_path));

  storage_options_.append_files = true;
  auto sequential_writer = create_sequential_writer();
  auto writer_ptr = sequential_writer.get();
  auto writer = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));
  writer->open(storage_options_, {"", ""});
  writer->split_bagfile();

  auto meta = writer_ptr->get_metadata();
  EXPECT_THAT(meta.relative_file_paths, SizeIs(3));
  EXPECT_EQ(no_ext(meta.relative_file_paths[0]), "uri_2");
  EXPECT_EQ(no_ext(meta.relative_file_paths[1]), "uri_3");
  EXPECT_EQ(no_ext(meta.relative_file_paths[2]), "uri_4");
}
