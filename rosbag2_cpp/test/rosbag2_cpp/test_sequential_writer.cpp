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

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"

#include "fake_data.hpp"
#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"
#include "mock_message_cache.hpp"
#include "mock_cache_consumer.hpp"

using namespace testing;  // NOLINT
using rosbag2_test_common::ParametrizedTemporaryDirectoryFixture;
namespace fs = std::filesystem;

class SequentialWriterForTest : public rosbag2_cpp::writers::SequentialWriter
{
public:
  SequentialWriterForTest(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
    std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface> converter_factory,
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
  :rosbag2_cpp::writers::SequentialWriter(
      std::move(storage_factory), std::move(converter_factory), std::move(metadata_io)) {}

  /// \brief Sets the message cache and cache consumer.
  /// \note Needs to be called after the SequentialWriterForTest::open(..) method call.
  void set_message_cache_and_cache_consumer(
    std::shared_ptr<rosbag2_cpp::cache::MessageCacheInterface> message_cache,
    std::unique_ptr<rosbag2_cpp::cache::CacheConsumer> cache_consumer)
  {
    message_cache_ = std::move(message_cache);
    cache_consumer_ = std::move(cache_consumer);
  }
};

class SequentialWriterTest : public Test
{
public:
  SequentialWriterTest()
  {
    storage_factory_ = std::make_unique<StrictMock<MockStorageFactory>>();
    storage_ = std::make_shared<NiceMock<MockStorage>>();
    converter_factory_ = std::make_shared<StrictMock<MockConverterFactory>>();
    metadata_io_ = std::make_unique<NiceMock<MockMetadataIo>>();
    tmp_dir_ = fs::temp_directory_path() / "SequentialWriterTest";
    storage_options_ = rosbag2_storage::StorageOptions{};
    storage_options_.uri = (tmp_dir_ / bag_base_dir_).string();

    fs::remove_all(tmp_dir_);

    ON_CALL(*storage_factory_, open_read_write(_)).WillByDefault(
      DoAll(
        Invoke(
          [this](const rosbag2_storage::StorageOptions & storage_options) {
            fake_storage_size_ = 0;
            fake_storage_uri_ = storage_options.uri;
          }),
        Return(storage_)));
    EXPECT_CALL(*storage_factory_, open_read_write(_)).Times(AtLeast(0));

    // intercept the metadata write so we can analyze it.
    ON_CALL(*storage_, update_metadata).WillByDefault(
      [this](const rosbag2_storage::BagMetadata & metadata) {
        v_intercepted_update_metadata_.emplace_back(metadata);
      });
    ON_CALL(*storage_, set_read_order).WillByDefault(Return(true));

    ON_CALL(*storage_,
            write_message(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>()))
    .WillByDefault(
      [this](std::shared_ptr<const rosbag2_storage::SerializedBagMessage>) {
        fake_storage_size_ += 1;
        return true;
      }
    );

    ON_CALL(*storage_, write_messages(An<const rosbag2_storage::SerializedBagMessages &>()))
    .WillByDefault(
      [this](const rosbag2_storage::SerializedBagMessages & msgs) {
        fake_storage_size_.fetch_add(static_cast<uint32_t>(msgs.size()));
        std::vector<size_t> lost_messages;
        return lost_messages;
      }
    );

    // intercept the metadata write so we can analyze it.
    ON_CALL(*metadata_io_, write_metadata).WillByDefault(
      [this](const std::string &, const rosbag2_storage::BagMetadata & metadata) {
        fake_metadata_ = metadata;
      }
    );

    ON_CALL(*storage_, get_bagfile_size).WillByDefault(
      [this]() {
        return fake_storage_size_.load();
      }
    );

    ON_CALL(*storage_, get_relative_file_path).WillByDefault(
      [this]() {
        return fake_storage_uri_;
      }
    );
  }

  ~SequentialWriterTest() override
  {
    fs::remove_all(tmp_dir_);
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
  std::vector<rosbag2_storage::BagMetadata> v_intercepted_update_metadata_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string fake_storage_uri_;
  const std::string bag_base_dir_ = "test_bag";
  size_t num_messages_to_lose_ = 0;

  /// \brief Checks if the filename in the given path matches the expected pattern for a bag file
  /// created by SequentialWriter.
  /// \details Strips the directory and extension from the path, then checks if the filename starts
  /// with the expected counter and bag base directory, followed by a timestamp in
  /// the "YYYY_MM_DD-HH_MM_SS" format. The expected filename pattern is constructed as follows:
  /// {expected_counter}_{prefix}_{timestamp}.extension
  /// \param path the full path to the bag file
  /// \param prefix the expected bag base directory prefix in the filename
  /// \param expected_counter the expected counter value that should be at the start of the filename
  /// \return true if the filename matches the expected pattern, false otherwise
  static bool matches_filename_pattern(
    const std::string & path, const std::string & prefix, size_t expected_counter)
  {
    std::string filename_no_ext = fs::path(fs::path(path).filename()).stem().generic_string();
    std::string expected_prefix = std::to_string(expected_counter) + "_" + prefix + "_";
    // Check if filename starts with expected prefix
    if (filename_no_ext.size() < expected_prefix.size() ||
      filename_no_ext.substr(0, expected_prefix.size()) != expected_prefix)
    {
      return false;
    }
    // Extract the timestamp part of the filename and validate its format
    size_t timestamp_start = expected_prefix.size();
    const std::string kTimestampFormat = "YYYY_MM_DD-HH_MM_SS";
    const size_t kTimestampLength = kTimestampFormat.length();
    if (filename_no_ext.size() < timestamp_start + kTimestampLength) {
      return false;
    }
    std::string timestamp = filename_no_ext.substr(timestamp_start, kTimestampLength);
    static const std::regex timestamp_pattern(rosbag2_cpp::writers::TIMESTAMP_PATTERN);
    return std::regex_match(timestamp, timestamp_pattern);
  }

  static size_t extract_counter_from_filename(const std::string & path)
  {
    std::string filename = fs::path(path).filename().generic_string();
    static const std::regex pattern("^(\\d+)_");
    std::smatch match;
    if (std::regex_search(filename, match, pattern)) {
      return std::stoull(match[1].str());
    }
    return SIZE_MAX;
  }

  static std::string extract_timestamp_from_filename(const std::string & path)
  {
    std::string filename_no_ext = fs::path(fs::path(path).filename()).stem().generic_string();
    static std::regex pattern("_" + std::string(rosbag2_cpp::writers::TIMESTAMP_PATTERN) + "$");
    std::smatch match;
    if (std::regex_search(filename_no_ext, match, pattern)) {
      return match[0].str().substr(1);  // Remove leading underscore
    }
    return "";
  }

  static bool timestamp_matches(
    const std::string & filename_timestamp,
    const std::chrono::system_clock::time_point & recorded_time,
    std::chrono::seconds tolerance = std::chrono::seconds(5))
  {
    std::tm tm_parsed = {};
    std::istringstream ss(filename_timestamp);
    ss >> std::get_time(&tm_parsed, "%Y_%m_%d-%H_%M_%S");
    if (ss.fail()) {
      return false;
    }
    tm_parsed.tm_isdst = -1;
    auto filename_time_t = std::mktime(&tm_parsed);
    auto filename_time_point = std::chrono::system_clock::from_time_t(filename_time_t);
    auto recorded_time_t = std::chrono::system_clock::to_time_t(recorded_time);
    auto recorded_time_point = std::chrono::system_clock::from_time_t(recorded_time_t);
    auto diff = (filename_time_point > recorded_time_point) ?
      (filename_time_point - recorded_time_point) : (recorded_time_point - filename_time_point);
    return diff <= tolerance;
  }
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

TEST_F(SequentialWriterTest, create_topic_does_not_throw_if_writer_not_open) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  rosbag2_storage::TopicMetadata topic_metadata{0u, "topic1", "test_msgs/BasicTypes", "", {}, ""};

  EXPECT_NO_THROW(writer_->create_topic(topic_metadata));
}

TEST_F(SequentialWriterTest, remove_topic_does_not_throw_if_writer_not_open) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  rosbag2_storage::TopicMetadata topic_metadata{0u, "topic1", "test_msgs/BasicTypes", "", {}, ""};

  EXPECT_NO_THROW(writer_->create_topic(topic_metadata));
  EXPECT_NO_THROW(writer_->remove_topic(topic_metadata));
}

TEST_F(SequentialWriterTest, topics_persist_between_close_and_open) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  auto test_message = make_test_msg();
  rosbag2_storage::TopicMetadata topic_metadata {
    0U, test_message->topic_name, "test_msgs/BasicTypes", "", {}, ""
  };

  writer_->create_topic(topic_metadata);
  writer_->open(storage_options_);
  writer_->close();

  // Reopen the writer and verify the topic still exists. i.e., writing a message without
  // recreating the topic.
  storage_options_.uri += "(1)";  // Add suffix to create a new bag folder in the scope of the test
  writer_->open(storage_options_);
  EXPECT_NO_THROW(writer_->write(test_message));
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
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});
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
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});
  writer_->write(message);
}

TEST_F(SequentialWriterTest, writer_uses_correct_serialization_format_in_metadata) {
  // First open writer with conversion and verify metadata uses the output serialization format.
  // Then open again with no conversion and verify metadata uses the input serialization format
  // for both yaml file and storage.
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string storage_serialization_format = "rmw2_format";
  std::string input_format = "rmw1_format";

  auto format1_converter = std::make_unique<StrictMock<MockConverter>>();
  auto format2_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(*format1_converter, serialize(_, _, _)).Times(2);
  EXPECT_CALL(*format2_converter, deserialize(_, _, _)).Times(2);

  EXPECT_CALL(*converter_factory_, load_serializer(storage_serialization_format))
  .WillOnce(Return(ByMove(std::move(format1_converter))));
  EXPECT_CALL(*converter_factory_, load_deserializer(input_format))
  .WillOnce(Return(ByMove(std::move(format2_converter))));

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";
  std::string msg_content = "Hello";
  message->serialized_data =
    rosbag2_storage::make_serialized_message(msg_content.c_str(), msg_content.length());

  writer_->create_topic({0u, "test_topic", "test_msgs/msg/BasicTypes", input_format, {}, ""});
  writer_->open(storage_options_, {input_format, storage_serialization_format});
  writer_->write(message);
  writer_->write(message);
  writer_->close();

  ASSERT_EQ(fake_metadata_.topics_with_message_count.size(), 1u);
  EXPECT_EQ(fake_metadata_.topics_with_message_count[0].topic_metadata.serialization_format,
            storage_serialization_format);

  for (const auto & metadata : v_intercepted_update_metadata_) {
    ASSERT_EQ(metadata.topics_with_message_count.size(), 1u);
    EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.serialization_format,
              storage_serialization_format);
  }

  // Open again with no conversion and verify metadata uses the input serialization format
  storage_options_.uri += "(1)";  // Add suffix to create a new bag folder in the scope of the test
  writer_->open(storage_options_, {input_format, input_format});
  writer_->write(message);
  const auto & metadata = v_intercepted_update_metadata_.back();
  ASSERT_EQ(metadata.topics_with_message_count.size(), 1u);
  EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.serialization_format,
            input_format);

  writer_->close();

  ASSERT_EQ(fake_metadata_.topics_with_message_count.size(), 1u);
  EXPECT_EQ(fake_metadata_.topics_with_message_count[0].topic_metadata.serialization_format,
            input_format);
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

TEST_F(SequentialWriterTest, sequantial_writer_call_metadata_update_on_open_and_destruction)
{
  const std::string test_topic_name = "test_topic";
  const std::string test_topic_type = "test_msgs/BasicTypes";
  EXPECT_CALL(*storage_, update_metadata(_)).Times(2);

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";
  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({0u, test_topic_name, test_topic_type, "", {}, ""});

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = test_topic_name;

  const size_t kNumMessagesToWrite = 5;
  for (size_t i = 0; i < kNumMessagesToWrite; i++) {
    writer_->write(message);
  }
  writer_.reset();  // reset will call writer destructor

  EXPECT_EQ(v_intercepted_update_metadata_.size(), 2u);
  EXPECT_TRUE(v_intercepted_update_metadata_[0].compression_mode.empty());
  EXPECT_EQ(v_intercepted_update_metadata_[0].message_count, 0u);
  EXPECT_EQ(v_intercepted_update_metadata_[1].message_count, kNumMessagesToWrite);
}

TEST_F(SequentialWriterTest, sequantial_writer_call_metadata_update_on_bag_split)
{
  const std::string test_topic_name = "test_topic";
  const std::string test_topic_type = "test_msgs/BasicTypes";
  EXPECT_CALL(*storage_, update_metadata(_)).Times(4);

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";
  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({0u, test_topic_name, test_topic_type, "", {}, ""});

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = test_topic_name;

  const size_t kNumMessagesToWrite = 5;
  for (size_t i = 0; i < kNumMessagesToWrite; i++) {
    writer_->write(message);
  }

  writer_->split_bagfile();

  for (size_t i = 0; i < kNumMessagesToWrite; i++) {
    writer_->write(message);
  }
  writer_.reset();  // reset will call writer destructor

  ASSERT_EQ(v_intercepted_update_metadata_.size(), 4u);
  EXPECT_TRUE(v_intercepted_update_metadata_[0].compression_mode.empty());
  EXPECT_EQ(v_intercepted_update_metadata_[0].message_count, 0u);  // On opening first bag file
  EXPECT_EQ(v_intercepted_update_metadata_[1].files.size(), 1u);   // On closing first bag file
  EXPECT_EQ(v_intercepted_update_metadata_[2].files.size(), 2u);   // On opening second bag file
  EXPECT_EQ(v_intercepted_update_metadata_[3].files.size(), 2u);   // On writer destruction
  EXPECT_EQ(v_intercepted_update_metadata_[3].message_count, 2 * kNumMessagesToWrite);
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
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

  for (auto i = 0; i < counter; ++i) {
    writer_->write(message);
  }
}

TEST_F(SequentialWriterTest, writer_splits_when_storage_bagfile_size_gt_max_bagfile_size) {
  const int message_count = 15;
  const int max_bagfile_size = 5;
  const auto expected_splits = message_count / max_bagfile_size;
  fake_storage_size_ = 0;

  EXPECT_CALL(*metadata_io_, write_metadata).Times(1);

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";

  storage_options_.max_bagfile_size = max_bagfile_size;

  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

  for (auto i = 0; i < message_count; ++i) {
    writer_->write(message);
  }

  writer_.reset();
  // metadata should be written now that the Writer was released.

  EXPECT_EQ(
    fake_metadata_.relative_file_paths.size(),
    static_cast<unsigned int>(expected_splits)) <<
    "Storage should have split bagfile " << (expected_splits - 1);

  size_t counter = 0;
  for (const auto & path : fake_metadata_.relative_file_paths) {
    EXPECT_TRUE(matches_filename_pattern(path, bag_base_dir_, counter))
      << "Filename '" << path << "' does not match expected pattern";
    EXPECT_EQ(extract_counter_from_filename(path), counter)
      << "Counter in filename '" << path << "' does not match expected value";
    counter++;
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

  ON_CALL(*storage_, write_messages(An<const rosbag2_storage::SerializedBagMessages &>()))
  .WillByDefault(
    [this, &written_messages](const rosbag2_storage::SerializedBagMessages & msgs)
    {
      written_messages += msgs.size();
      fake_storage_size_.fetch_add(static_cast<uint32_t>(msgs.size()));
      std::vector<size_t> lost_messages;
      return lost_messages;
    });

  EXPECT_CALL(*metadata_io_, write_metadata).Times(1);

  EXPECT_CALL(*storage_factory_, open_read_write(_)).Times(3);

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  std::string rmw_format = "rmw_format";

  storage_options_.max_bagfile_size = max_bagfile_size;
  storage_options_.max_cache_size = 4000u;
  storage_options_.snapshot_mode = false;

  writer_->open(storage_options_, {rmw_format, rmw_format});
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

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

  size_t counter = 0;
  for (const auto & path : fake_metadata_.relative_file_paths) {
    EXPECT_TRUE(matches_filename_pattern(path, bag_base_dir_, counter))
      << "Filename '" << path << "' does not match expected pattern";
    EXPECT_EQ(extract_counter_from_filename(path), counter)
      << "Counter in filename '" << path << "' does not match expected value";
    counter++;
  }
}

TEST_F(SequentialWriterTest, do_not_use_cache_if_cache_size_is_zero) {
  const size_t counter = 1000;
  const uint64_t max_cache_size = 0;

  EXPECT_CALL(*storage_,
              write_messages(An<const rosbag2_storage::SerializedBagMessages &>())).Times(0);

  EXPECT_CALL(*storage_,
              write_message(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>()))
  .Times(counter);

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
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

  for (auto i = 0u; i < counter; ++i) {
    writer_->write(message);
  }
}

TEST_F(SequentialWriterTest, snapshot_mode_write_on_trigger)
{
  storage_options_.max_bagfile_size = 0;
  storage_options_.max_cache_size = 700;
  storage_options_.snapshot_mode = true;

  // Expect a single write call when the snapshot is triggered
  EXPECT_CALL(*storage_,
              write_messages(An<const rosbag2_storage::SerializedBagMessages &>())).Times(1);

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
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

  for (auto i = 0u; i < 100; ++i) {
    writer_->write(message);
  }
  writer_->take_snapshot();
}

TEST_F(SequentialWriterTest, snapshot_mode_not_triggered_no_storage_write)
{
  storage_options_.max_bagfile_size = 0;
  storage_options_.max_cache_size = 700;
  storage_options_.snapshot_mode = true;

  // Storage should never be written to when snapshot mode is enabled
  // but a snapshot is never triggered
  EXPECT_CALL(*storage_,
              write_messages(An<const rosbag2_storage::SerializedBagMessages &>())).Times(0);

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
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

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
  storage_options_.max_cache_size = 700;
  storage_options_.snapshot_mode = true;
  const rcutils_time_point_value_t first_msg_timestamp = 100;
  const size_t num_msgs_to_write = 150;
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
    message->recv_timestamp = first_msg_timestamp + static_cast<rcutils_time_point_value_t>(i);
    message->send_timestamp = first_msg_timestamp + static_cast<rcutils_time_point_value_t>(i);
    message->topic_name = topic_name;
    message->serialized_data =
      rosbag2_storage::make_serialized_message(msg_content.c_str(), msg_content.length());
    messages.push_back(message);
  }

  // Expect a single write call when the snapshot is triggered
  EXPECT_CALL(*storage_,
              write_messages(An<const rosbag2_storage::SerializedBagMessages &>())).Times(1);
  EXPECT_CALL(*storage_,
              write_message(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>()))
  .Times(0);

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
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

  for (const auto & message : messages) {
    writer_->write(message);
  }
  writer_->take_snapshot();

  EXPECT_THAT(closed_files.size(), 1);
  EXPECT_THAT(opened_files.size(), 1);

  if (!((closed_files.size() == opened_files.size()) && (opened_files.size() == 1))) {
    // Output debug info
    for (size_t i = 0; i < opened_files.size(); i++) {
      std::cout << "opened_file[" << i << "] = '" << opened_files[i] <<
        "'; closed_file[" << i << "] = '" << closed_files[i] << "';" << std::endl;
    }
  }

  ASSERT_EQ(opened_files.size(), 1);
  ASSERT_EQ(closed_files.size(), 1);

  // Check that filenames match the new naming pattern
  EXPECT_TRUE(matches_filename_pattern(closed_files[0], bag_base_dir_, 0))
    << "Closed filename '" << closed_files[0] << "' does not match expected pattern";
  EXPECT_EQ(extract_counter_from_filename(closed_files[0]), 0u)
    << "Counter in closed filename '" << closed_files[0] << "' does not match expected value";
  EXPECT_TRUE(matches_filename_pattern(opened_files[0], bag_base_dir_, 1))
    << "Opened filename '" << opened_files[0] << "' does not match expected pattern";
  EXPECT_EQ(extract_counter_from_filename(opened_files[0]), 1u)
    << "Counter in opened filename '" << opened_files[0] << "' does not match expected value";

  // Check metadata
  ASSERT_EQ(v_intercepted_update_metadata_.size(), 3u);
  // The v_intercepted_update_metadata_[0] is the very first metadata saved from the writer's
  // constructor. We don't update it during the snapshot, and it doesn't make sense checking it.
  // The v_intercepted_update_metadata_[1] is the metadata written right before closing the file
  // with the new snapshot.
  // The v_intercepted_update_metadata_[2] is the metadata written when we are opening a new file
  // after switching to a new storage.
  EXPECT_EQ(v_intercepted_update_metadata_[1].message_count, num_expected_msgs);
  EXPECT_EQ(v_intercepted_update_metadata_[2].message_count, num_expected_msgs);
  EXPECT_EQ(
    std::chrono::time_point_cast<std::chrono::nanoseconds>(
      v_intercepted_update_metadata_[1].starting_time).time_since_epoch().count(),
    first_msg_timestamp);

  ASSERT_FALSE(v_intercepted_update_metadata_[1].files.empty());
  const auto & first_file_info = v_intercepted_update_metadata_[1].files[0];
  EXPECT_TRUE(matches_filename_pattern(first_file_info.path, bag_base_dir_, 0))
    << "First file path '" << first_file_info.path << "' does not match expected pattern";
  EXPECT_EQ(first_file_info.message_count, num_expected_msgs);
  EXPECT_EQ(
    std::chrono::time_point_cast<std::chrono::nanoseconds>(
    first_file_info.starting_time).time_since_epoch().count(),
    expected_start_time);
  EXPECT_EQ(first_file_info.duration.count(), expected_duration);
}

TEST_F(SequentialWriterTest, snapshot_can_be_called_twice)
{
  storage_options_.max_bagfile_size = 0;
  storage_options_.max_cache_size = 700;
  storage_options_.snapshot_mode = true;
  const size_t num_msgs_to_write = 100;

  // Expect to call write method twice. Once per each snapshot.
  EXPECT_CALL(*storage_,
              write_messages(An<const rosbag2_storage::SerializedBagMessages &>())).Times(2);

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
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

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
    EXPECT_TRUE(matches_filename_pattern(closed_files[i], bag_base_dir_, i))
      << "Closed filename '" << closed_files[i] << "' does not match expected pattern";
    EXPECT_EQ(extract_counter_from_filename(closed_files[i]), i)
      << "Counter in closed filename '" << closed_files[i] << "' does not match expected value";
    EXPECT_TRUE(matches_filename_pattern(opened_files[i], bag_base_dir_, i + 1))
      << "Opened filename '" << opened_files[i] << "' does not match expected pattern";
    EXPECT_EQ(extract_counter_from_filename(opened_files[i]), i + 1)
      << "Counter in opened filename '" << opened_files[i] << "' does not match expected value";
  }
}

TEST_F(SequentialWriterTest, calls_callback_on_storage_message_lost_with_no_cache)
{
  const size_t message_count = 10;
  const size_t expected_lost_messages = 4;
  const size_t expected_written_messages = message_count - expected_lost_messages;
  num_messages_to_lose_ = expected_lost_messages;
  EXPECT_CALL(*storage_,
              write_message(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>()))
  .Times(message_count);

  ON_CALL(*storage_,
    write_message(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>()))
  .WillByDefault(
    Invoke(
      [this](const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> &) -> bool {
        static size_t curr_number_of_lost_messages = 0;
        if (curr_number_of_lost_messages < num_messages_to_lose_) {
          curr_number_of_lost_messages++;
          return false;  // Simulate message loss
        } else {
          fake_storage_size_++;
          return true;  // Simulate successful write
        }
      }
    )
  );

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";

  storage_options_.max_cache_size = 0;  // Disable cache

  size_t num_lost_messages = 0;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.messages_lost_callback =
    [&num_lost_messages](const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> &
    msgs_lost_info) {
      for (const auto & info : msgs_lost_info) {
        num_lost_messages += info.num_messages_lost;
      }
    };
  writer_->add_event_callbacks(callbacks);

  writer_->open(storage_options_, {"rmw_format", "rmw_format"});
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

  for (size_t i = 0; i < message_count; i++) {
    writer_->write(message);
  }
  writer_->close();

  EXPECT_EQ(num_lost_messages, expected_lost_messages);
  EXPECT_EQ(fake_metadata_.message_count, expected_written_messages);
  ASSERT_EQ(fake_metadata_.files.size(), 1u);
  EXPECT_EQ(fake_metadata_.files[0].message_count, expected_written_messages);
  ASSERT_EQ(fake_metadata_.topics_with_message_count.size(), 1u);
  EXPECT_EQ(fake_metadata_.topics_with_message_count[0].message_count, expected_written_messages);
}

TEST_F(SequentialWriterTest, calls_callback_on_storage_messages_lost_with_cache)
{
  const size_t message_count = 10;
  const size_t expected_lost_messages = 5;
  const size_t expected_written_messages = message_count - expected_lost_messages;
  num_messages_to_lose_ = expected_lost_messages;

  EXPECT_CALL(*storage_,
              write_message(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>()))
  .Times(0);    // No direct writes to storage

  ON_CALL(*storage_, write_messages(An<const rosbag2_storage::SerializedBagMessages &>()))
  .WillByDefault(
    Invoke(
      [this](const rosbag2_storage::SerializedBagMessages & msgs)
      {
        static size_t curr_number_of_lost_messages = 0;
        std::vector<size_t> lost_messages;
        for (size_t i = 0; i < msgs.size(); i++) {
          if (curr_number_of_lost_messages < num_messages_to_lose_) {
            lost_messages.push_back(i);    // Simulate message loss
            curr_number_of_lost_messages++;
          } else {
            fake_storage_size_++;
          }
        }
        return lost_messages;
      }
    )
  );

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  auto message_to_write = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message_to_write->topic_name = "test_topic";
  std::string msg_content = "Hello";
  message_to_write->serialized_data =
    rosbag2_storage::make_serialized_message(msg_content.c_str(), msg_content.length());

  storage_options_.max_cache_size = 100 * 1024;  // Enable cache 100 KiB

  size_t num_lost_messages = 0;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.messages_lost_callback =
    [&num_lost_messages](
    const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> & msgs_lost_info) {
      for (const auto & info : msgs_lost_info) {
        num_lost_messages += info.num_messages_lost;
      }
    };
  writer_->add_event_callbacks(callbacks);

  writer_->open(storage_options_, {"rmw_format", "rmw_format"});
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

  for (size_t i = 0; i < message_count; i++) {
    writer_->write(message_to_write);
  }
  writer_->close();

  EXPECT_EQ(num_lost_messages, expected_lost_messages);
  EXPECT_EQ(fake_metadata_.message_count, expected_written_messages);
  ASSERT_EQ(fake_metadata_.files.size(), 1u);
  EXPECT_EQ(fake_metadata_.files[0].message_count, expected_written_messages);
  ASSERT_EQ(fake_metadata_.topics_with_message_count.size(), 1u);
  EXPECT_EQ(fake_metadata_.topics_with_message_count[0].message_count, expected_written_messages);
}

TEST_F(SequentialWriterTest, calls_callback_on_messages_loss_in_writer_cache)
{
  const size_t message_count = 10;
  const size_t expected_lost_messages = 5;
  num_messages_to_lose_ = expected_lost_messages;

  auto sequential_writer = std::make_unique<SequentialWriterForTest>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));

  size_t num_lost_messages = 0;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.messages_lost_callback =
    [&num_lost_messages](
    const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> & msgs_lost_info) {
      for (const auto & info : msgs_lost_info) {
        num_lost_messages += info.num_messages_lost;
      }
    };
  sequential_writer->add_event_callbacks(callbacks);

  storage_options_.max_cache_size = 100 * 1024;  // Enable cache 100 KiB
  sequential_writer->open(storage_options_, {"rmw_format", "rmw_format"});

  size_t consumed_message_count = 0;
  auto mock_message_cache = std::make_shared<NiceMock<MockMessageCache>>(1024u);
  auto write_messages_cb = [&consumed_message_count](
    const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msgs) {
      consumed_message_count += msgs.size();
    };
  mock_message_cache->use_mock_push(true);

  // Simulate messages loss in the cache push method
  ON_CALL(*mock_message_cache,
        mock_push(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>()))
  .WillByDefault(
    Invoke(
      [this](const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> &)-> bool
      {
        static size_t num_lost_messages_in_cache = 0;
        if (num_lost_messages_in_cache < num_messages_to_lose_) {
          num_lost_messages_in_cache++;
          return false;
        } else {
          return true;
        }
      }
    )
  );

  auto mock_cache_consumer =
    std::make_unique<NiceMock<MockCacheConsumer>>(mock_message_cache, write_messages_cb);

  sequential_writer->set_message_cache_and_cache_consumer(
    mock_message_cache, std::move(mock_cache_consumer));

  sequential_writer->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

  auto message_to_write = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message_to_write->topic_name = "test_topic";
  std::string msg_content = "Hello";
  message_to_write->serialized_data =
    rosbag2_storage::make_serialized_message(msg_content.c_str(), msg_content.length());

  for (size_t i = 0; i < message_count; i++) {
    sequential_writer->write(message_to_write);
  }
  sequential_writer->close();

  EXPECT_EQ(num_lost_messages, expected_lost_messages);
}

TEST_F(SequentialWriterTest, split_event_calls_callback)
{
  const uint64_t max_bagfile_size = 3;
  const size_t num_splits = 2;
  const int message_count = max_bagfile_size * num_splits + max_bagfile_size - 1;  // 8

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
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

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
    EXPECT_TRUE(matches_filename_pattern(closed_files[i], bag_base_dir_, i))
      << "Closed filename '" << closed_files[i] << "' does not match expected pattern";
    EXPECT_EQ(extract_counter_from_filename(closed_files[i]), i)
      << "Counter in closed filename '" << closed_files[i] << "' does not match expected value";

    if (i == num_splits) {
      // The last opened file shall be empty string when we do "writer->close();"
      EXPECT_TRUE(opened_files[i].empty());
    } else {
      EXPECT_TRUE(matches_filename_pattern(opened_files[i], bag_base_dir_, i + 1))
        << "Opened filename '" << opened_files[i] << "' does not match expected pattern";
      EXPECT_EQ(extract_counter_from_filename(opened_files[i]), i + 1)
        << "Counter in opened filename '" << opened_files[i] << "' does not match expected value";
    }
  }
}

TEST_F(SequentialWriterTest, split_event_calls_on_writer_close)
{
  const int message_count = 7;

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
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

  for (auto i = 0; i < message_count; ++i) {
    writer_->write(message);
  }
  writer_->close();

  ASSERT_TRUE(callback_called);
  EXPECT_TRUE(matches_filename_pattern(closed_file, bag_base_dir_, 0))
    << "Closed filename '" << closed_file << "' does not match expected pattern";
  EXPECT_EQ(extract_counter_from_filename(closed_file), 0u)
    << "Counter in closed filename '" << closed_file << "' does not match expected value";
  EXPECT_TRUE(opened_file.empty());
}

TEST_F(SequentialWriterTest, split_prepends_transient_local_messages_to_next_bag)
{
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  storage_options_.max_cache_size = 0;
  writer_->open(storage_options_, {"rmw_format", "rmw_format"});

  const std::string topic_name = "latched_topic";
  writer_->create_transient_local_topic({0u, topic_name, "test_msgs/BasicTypes", "", {}, ""}, 1);

  auto first_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  first_message->topic_name = topic_name;
  first_message->recv_timestamp = 100;
  first_message->send_timestamp = 100;

  auto second_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  second_message->topic_name = topic_name;
  second_message->recv_timestamp = 200;
  second_message->send_timestamp = 200;

  rosbag2_storage::SerializedBagMessages prepended_messages;
  // Expected to call storage_->write_messages(msgs) once from write_transient_local_messages()
  // because we configured writer to not use cache and messages before split will be written with
  // via storage_->write_message(msg) directly.
  EXPECT_CALL(*storage_, write_messages(An<const rosbag2_storage::SerializedBagMessages &>()))
  .WillOnce(Invoke(
      [&prepended_messages](const rosbag2_storage::SerializedBagMessages & messages)
      {
        prepended_messages = messages;
        return std::vector<size_t>{};
      })
  );
  EXPECT_CALL(*storage_,
    write_message(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>())).Times(2);

  writer_->write(first_message);
  writer_->write(second_message);

  writer_->split_bagfile();

  ASSERT_EQ(prepended_messages.size(), 1u);
  EXPECT_EQ(prepended_messages[0]->topic_name, topic_name);
  EXPECT_EQ(prepended_messages[0]->recv_timestamp, 200);
  EXPECT_EQ(prepended_messages[0]->send_timestamp, 200);
}

TEST_F(SequentialWriterTest, snapshot_merge_prepends_transient_local_messages)
{
  rosbag2_storage::SerializedBagMessages written_msgs;

  EXPECT_CALL(*storage_, write_messages(An<const rosbag2_storage::SerializedBagMessages &>()))
  .WillOnce(Invoke(
      [&written_msgs](const rosbag2_storage::SerializedBagMessages & messages)
      {
        written_msgs = messages;
        return std::vector<size_t>{};
      })
  );

  auto sequential_writer = std::make_unique<SequentialWriterForTest>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));

  storage_options_.max_cache_size = 0;  // Limit cache only by duration
  storage_options_.max_cache_duration = 1;  // Limit cache duration to 1 second
  storage_options_.snapshot_mode = true;
  sequential_writer->open(storage_options_, {"rmw_format", "rmw_format"});

  const std::string latched_topic = "latched_topic";
  const std::string data_topic = "data_topic";

  sequential_writer->create_transient_local_topic(
    {0u, latched_topic, "test_msgs/BasicTypes", "", {}, ""}, 2);
  sequential_writer->create_topic({0u, data_topic, "test_msgs/BasicTypes", "", {}, ""});

  // Write a latched message to populate the transient local cache
  auto latched_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  latched_msg->topic_name = latched_topic;
  latched_msg->recv_timestamp = 100;
  latched_msg->send_timestamp = 100;
  latched_msg->serialized_data = rosbag2_storage::make_serialized_message("A", 1);
  sequential_writer->write(latched_msg);

  // Simulate a snapshot buffer where the latched message was evicted due to cache duration limit,
  // but still exists in the transient local cache.
  auto data_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  data_msg->topic_name = data_topic;
  data_msg->recv_timestamp = RCUTILS_S_TO_NS(2);
  data_msg->send_timestamp = RCUTILS_S_TO_NS(2);
  data_msg->serialized_data = rosbag2_storage::make_serialized_message("B", 1);
  sequential_writer->write(data_msg);

  latched_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  latched_msg->topic_name = latched_topic;
  latched_msg->recv_timestamp = RCUTILS_S_TO_NS(2.5);
  latched_msg->send_timestamp = RCUTILS_S_TO_NS(2.5);
  latched_msg->serialized_data = rosbag2_storage::make_serialized_message("C", 1);
  sequential_writer->write(latched_msg);

  data_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  data_msg->topic_name = data_topic;
  data_msg->recv_timestamp = RCUTILS_S_TO_NS(3);
  data_msg->send_timestamp = RCUTILS_S_TO_NS(3);
  data_msg->serialized_data = rosbag2_storage::make_serialized_message("D", 1);
  sequential_writer->write(data_msg);

  sequential_writer->take_snapshot();

  // Verify: Transient local message was merged in with adjusted timestamps.
  // Transient messages are prepended before snapshot messages, so transient messages comes first.
  ASSERT_EQ(written_msgs.size(), 4u);
  EXPECT_EQ(written_msgs[0]->topic_name, latched_topic);
  EXPECT_EQ(written_msgs[0]->recv_timestamp, RCUTILS_S_TO_NS(2));  // adjusted to T_earliest
  EXPECT_EQ(written_msgs[0]->send_timestamp, RCUTILS_S_TO_NS(2));

  EXPECT_EQ(written_msgs[1]->topic_name, data_topic);
  EXPECT_EQ(written_msgs[1]->recv_timestamp, RCUTILS_S_TO_NS(2));
  EXPECT_EQ(written_msgs[1]->send_timestamp, RCUTILS_S_TO_NS(2));

  EXPECT_EQ(written_msgs[2]->topic_name, latched_topic);
  EXPECT_EQ(written_msgs[2]->recv_timestamp, RCUTILS_S_TO_NS(2.5));
  EXPECT_EQ(written_msgs[2]->send_timestamp, RCUTILS_S_TO_NS(2.5));

  EXPECT_EQ(written_msgs[3]->topic_name, data_topic);
  EXPECT_EQ(written_msgs[3]->recv_timestamp, RCUTILS_S_TO_NS(3));
  EXPECT_EQ(written_msgs[3]->send_timestamp, RCUTILS_S_TO_NS(3));
}

TEST_F(SequentialWriterTest, circular_logging_limits_number_of_files_by_max_bag_files)
{
  // Configure frequent splits and a small retention window
  const uint64_t max_bagfile_size = 6;   // split every 6 writes (even for 2 topics)
  const uint64_t max_bag_files = 3;      // retain at most 3 files
  const int message_count = 30;          // enough writes to exceed retention

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  auto message1 = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message1->topic_name = "test_topic_1";
  auto message2 = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message2->topic_name = "test_topic_2";

  storage_options_.max_cache_size = 0;             // direct writes
  storage_options_.max_bagfile_size = max_bagfile_size;
  storage_options_.max_bag_files = max_bag_files;  // enable bag file count circular limit

  writer_->open(storage_options_, {"rmw_format", "rmw_format"});
  writer_->create_topic({0u, "test_topic_1", "test_msgs/BasicTypes", "", {}, ""});
  writer_->create_topic({0u, "test_topic_2", "test_msgs/BasicTypes", "", {}, ""});

  // Alternate messages between topics
  for (int i = 0; i < message_count; ++i) {
    writer_->write((i % 2 == 0) ? message1 : message2);
  }
  writer_->close();

  // After circular deletion, only max_bag_files files should remain in metadata
  ASSERT_LE(fake_metadata_.files.size(), max_bag_files);
  ASSERT_EQ(fake_metadata_.files.size(), fake_metadata_.relative_file_paths.size());

  // Verify message counts reflect only retained files (not full session totals)
  // 30 messages / 6 per file = 5 files total, 2 deleted, 3 retained with 6 messages each
  const size_t expected_total_count = max_bag_files * max_bagfile_size;  // 18 total
  const size_t expected_per_topic_count = expected_total_count / 2;       // 9 per topic
  EXPECT_EQ(fake_metadata_.message_count, expected_total_count);

  // Verify per-topic message counts are also adjusted correctly
  ASSERT_EQ(fake_metadata_.topics_with_message_count.size(), 2u);
  size_t topic1_count = 0, topic2_count = 0;
  for (const auto & topic_info : fake_metadata_.topics_with_message_count) {
    if (topic_info.topic_metadata.name == "test_topic_1") {
      topic1_count = topic_info.message_count;
    } else if (topic_info.topic_metadata.name == "test_topic_2") {
      topic2_count = topic_info.message_count;
    }
  }
  EXPECT_EQ(topic1_count, expected_per_topic_count);
  EXPECT_EQ(topic2_count, expected_per_topic_count);
}

TEST_P(
  ParametrizedTemporaryDirectoryFixture,
  circular_logging_deletes_oldest_files_with_real_storage)
{
  // Integration test: verify max_bag_files actually deletes files from disk
  const uint64_t max_bag_files = 3;
  const size_t total_files_to_create = 6;
  std::string topic_name = "circular_bag_topic";

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = (fs::path(temporary_dir_path_) / "circular_bag").generic_string();
  storage_options.storage_id = GetParam();
  storage_options.max_bag_files = max_bag_files;

  rosbag2_cpp::writers::SequentialWriter writer{};
  writer.open(storage_options, rosbag2_cpp::ConverterOptions{});
  rosbag2_storage::TopicMetadata topic_metadata {
    0U, topic_name, "test_msgs/BasicTypes", "cdr", {}, ""
  };
  writer.create_topic(topic_metadata);

  // Use manual splits instead of max_bagfile_size because storage plugins enforce
  // minimum split sizes (sqlite3: 86KB, mcap: 1KB) which would require large writes
  for (size_t i = 0; i < total_files_to_create; i++) {
    if (i > 0) {
      writer.split_bagfile();
    }

    auto msg = make_test_msg();
    msg->topic_name = topic_name;
    msg->recv_timestamp = static_cast<rcutils_time_point_value_t>(i * 100);
    msg->send_timestamp = msg->recv_timestamp;
    writer.write(msg);
  }
  writer.close();

  // Count bag files on disk
  size_t file_count = 0;
  const auto expected_ext = rosbag2_test_common::kTestedStorageIDsToExtensions.at(GetParam());
  for (const auto & entry : fs::directory_iterator(storage_options.uri)) {
    const auto ext = entry.path().extension().generic_string();
    if (ext == expected_ext) {
      file_count++;
    }
  }
  EXPECT_EQ(file_count, max_bag_files);

  // Verify metadata matches
  rosbag2_storage::MetadataIo metadata_io;
  auto metadata = metadata_io.read_metadata(storage_options.uri);
  EXPECT_EQ(metadata.relative_file_paths.size(), max_bag_files);
  EXPECT_EQ(metadata.files.size(), max_bag_files);
}

TEST_F(SequentialWriterTest, all_event_callbacks_can_be_installed) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  bool is_split_callback_called = false;
  bool is_messages_lost_callback_called = false;
  callbacks.write_split_callback =
    [&is_split_callback_called](rosbag2_cpp::bag_events::BagSplitInfo &) {
      is_split_callback_called = true;
    };
  callbacks.messages_lost_callback =
    [&is_messages_lost_callback_called](
    const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> &) {
      is_messages_lost_callback_called = true;
    };
  writer_->add_event_callbacks(callbacks);
  EXPECT_TRUE(writer_->has_callback_for_event(rosbag2_cpp::bag_events::BagEvent::WRITE_SPLIT));
  EXPECT_TRUE(writer_->has_callback_for_event(rosbag2_cpp::bag_events::BagEvent::MESSAGES_LOST));
}

TEST_F(SequentialWriterTest, add_event_callbacks_throws_if_callbacks_are_not_set) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  EXPECT_THROW(writer_->add_event_callbacks(callbacks);, std::runtime_error);
}

TEST_P(ParametrizedTemporaryDirectoryFixture, split_bag_metadata_has_full_duration) {
  const std::vector<std::pair<rcutils_time_point_value_t, uint32_t>> fake_messages {
    {100, 1},
    {300, 2},
    {200, 3},
    {500, 4},
    {400, 5},
    {600, 6}
  };
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri =
    (fs::path(temporary_dir_path_) / "split_duration_bag").generic_string();
  storage_options.storage_id = GetParam();
  write_sample_split_bag(storage_options, fake_messages, 3);

  rosbag2_storage::MetadataIo metadata_io;
  auto metadata = metadata_io.read_metadata(storage_options.uri);
  ASSERT_EQ(
    metadata.starting_time,
    std::chrono::high_resolution_clock::time_point(std::chrono::nanoseconds(100)));
  ASSERT_EQ(metadata.duration, std::chrono::nanoseconds(500));
}

TEST_F(SequentialWriterTest, filename_extracts_prefix_from_timestamped_directory_name)
{
  // Test that timestamp pattern is removed from directory name when generating filename
  // This simulates the case where:
  // * --output is not specified
  // * default timestamped directory is used
  const std::string timestamped_dir = "rosbag2_2025_11_04-20_21_42";

  EXPECT_CALL(*metadata_io_, write_metadata).Times(1);

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  writer_->open((tmp_dir_ / timestamped_dir).string());
  writer_->create_topic({0u, "test_topic", "test_msgs/BasicTypes", "", {}, ""});

  auto message = make_test_msg();
  writer_->write(message);

  // Record end timestamp after write operation completes,
  // for tolerance-based comparison with filename timestamp
  auto recorded_time = std::chrono::system_clock::now();
  writer_.reset();

  // Verify that filename uses "rosbag2" as prefix (timestamp pattern removed)
  ASSERT_FALSE(fake_metadata_.relative_file_paths.empty());
  const auto & path = fake_metadata_.relative_file_paths[0];

  EXPECT_TRUE(matches_filename_pattern(path, "rosbag2", 0))
    << "Filename '" << path << "' should use 'rosbag2' as prefix (timestamp pattern removed)";

  // Verify timestamp is present in filename
  std::string filename_timestamp = extract_timestamp_from_filename(path);
  ASSERT_FALSE(filename_timestamp.empty()) << "Filename should contain timestamp";
  EXPECT_TRUE(timestamp_matches(filename_timestamp, recorded_time))
    << "Timestamp in filename '" << filename_timestamp << "' should match recorded time";
}

INSTANTIATE_TEST_SUITE_P(
  SplitMetadataTest,
  ParametrizedTemporaryDirectoryFixture,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
