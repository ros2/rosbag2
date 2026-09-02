// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>

#include "rclcpp/time.hpp"

#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"
#include "rosbag2_transport/bag_rewrite.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

#include "test_msgs/msg/basic_types.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

namespace fs = std::filesystem;

/*
Builtin knowledge about the bags under test:
rewriter_a:
  - a_empty:
    - test_msgs/Empty
    - 100 messages
    - 2 offered QoS profiles
  - b_basictypes:
    - test_msgs/BasicTypes
    - 50 messages
    - 1 offered QoS profile
rewriter_b:
  - a_empty:
    - test_msgs/Empty
    - 25 messages
    - 1 offered Qos Profile
  - c_strings:
    - test_msgs/Strings
    - 50 messages
    - 1 offered QoS Profile
*/
class TestRewrite : public ParametrizedTemporaryDirectoryFixture
{
public:
  TestRewrite()
  {
    output_dir_ = fs::path(temporary_dir_path_);
    storage_id_ = GetParam();
    bags_path_ = fs::path(_SRC_RESOURCES_DIR_PATH) / storage_id_;
  }

  void use_input_a()
  {
    rosbag2_storage::StorageOptions storage;
    storage.uri = (bags_path_ / "rewriter_a").string();
    storage.storage_id = storage_id_;
    input_bags_.push_back(storage);
  }

  void use_input_b()
  {
    rosbag2_storage::StorageOptions storage;
    storage.uri = (bags_path_ / "rewriter_b").string();
    storage.storage_id = storage_id_;
    input_bags_.push_back(storage);
  }

  ~TestRewrite() override = default;

  fs::path output_dir_;
  fs::path bags_path_{_SRC_RESOURCES_DIR_PATH};
  std::string storage_id_;
  std::vector<rosbag2_storage::StorageOptions> input_bags_;
  std::vector<std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions>>
  output_bags_;
};

TEST_P(TestRewrite, test_noop_rewrite) {
  use_input_a();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "unchanged").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions output_record;
  output_record.all_topics = true;
  output_bags_.push_back({output_storage, output_record});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output_storage);
  reader->open(output_storage);
  const auto metadata = reader->get_metadata();
  EXPECT_EQ(metadata.message_count, 100u + 50u);
  EXPECT_THAT(metadata.topics_with_message_count, SizeIs(2));
  EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.serialization_format, "cdr");
}

TEST_P(TestRewrite, test_merge) {
  use_input_a();
  use_input_b();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "merged").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions output_record;
  output_record.all_topics = true;
  output_bags_.push_back({output_storage, output_record});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output_storage);
  reader->open(output_storage);
  const auto metadata = reader->get_metadata();
  EXPECT_EQ(metadata.message_count, 100u + 50u + 50u + 25u);

  // Check that all 3 topics are present (a_empty merged from both)
  EXPECT_THAT(metadata.topics_with_message_count, SizeIs(3));

  // Check that offered QoS profiles got concatenated
  for (const auto & topic_info : metadata.topics_with_message_count) {
    const auto topic = topic_info.topic_metadata;
    if (topic.name == "a_empty") {
      EXPECT_EQ(topic.offered_qos_profiles.size(), 3u);
    }
  }
}

TEST_P(TestRewrite, test_message_definitions_stored_with_merge) {
  use_input_a();
  use_input_b();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "merged").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions output_record;
  output_record.all_topics = true;
  output_bags_.push_back({output_storage, output_record});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output_storage);
  reader->open(output_storage);

  // Fill message_definitions_map
  std::vector<rosbag2_storage::MessageDefinition> msg_definitions;
  reader->get_all_message_definitions(msg_definitions);
  // Check that all 3 message definitions are present (a_empty merged from both)
  EXPECT_THAT(msg_definitions, SizeIs(3));
  bool found_strings_definition = false;
  bool found_empty_definition = false;
  bool found_basic_types_definition = false;
  for (const auto & msg_definition : msg_definitions) {
    if (msg_definition.topic_type == "test_msgs/msg/Empty") {
      EXPECT_TRUE(msg_definition.encoded_message_definition.empty());
      found_empty_definition = true;
    }
    if (msg_definition.topic_type == "test_msgs/msg/BasicTypes") {
      EXPECT_FALSE(msg_definition.encoded_message_definition.empty());
      found_basic_types_definition = true;
    }
    if (msg_definition.topic_type == "test_msgs/msg/Strings") {
      EXPECT_FALSE(msg_definition.encoded_message_definition.empty());
      found_strings_definition = true;
    }
  }
  EXPECT_TRUE(found_strings_definition);
  EXPECT_TRUE(found_empty_definition);
  EXPECT_TRUE(found_basic_types_definition);
}


TEST_P(TestRewrite, test_filter_split) {
  use_input_a();

  {
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = (output_dir_ / "split1").string();
    storage_opts.storage_id = storage_id_;
    rosbag2_transport::RecordOptions rec_opts;
    rec_opts.all_topics = true;
    rec_opts.exclude_regex = "basic";
    output_bags_.push_back({storage_opts, rec_opts});
  }
  {
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = (output_dir_ / "split2").string();
    storage_opts.storage_id = storage_id_;
    rosbag2_transport::RecordOptions rec_opts;
    rec_opts.all_topics = false;
    rec_opts.topics = {"b_basictypes"};
    output_bags_.push_back({storage_opts, rec_opts});
  }

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  {
    auto opts = output_bags_[0].first;
    auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(opts);
    reader->open(opts);
    const auto metadata = reader->get_metadata();
    EXPECT_THAT(metadata.topics_with_message_count, SizeIs(1));
    EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.name, "a_empty");
    EXPECT_EQ(metadata.message_count, 100u);
  }
  {
    auto opts = output_bags_[1].first;
    auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(opts);
    reader->open(opts);
    const auto metadata = reader->get_metadata();
    EXPECT_THAT(metadata.topics_with_message_count, SizeIs(1));
    EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.name, "b_basictypes");
    EXPECT_EQ(metadata.message_count, 50u);
  }
}

TEST_P(TestRewrite, test_compress) {
  use_input_a();

  rosbag2_storage::StorageOptions output_storage;
  auto out_bag = output_dir_ / "compressed";
  output_storage.uri = out_bag.string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions output_record;
  output_record.all_topics = true;
  output_record.compression_mode = "file";
  output_record.compression_format = "zstd";
  output_bags_.push_back({output_storage, output_record});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  rosbag2_storage::MetadataIo metadata_io;
  auto metadata = metadata_io.read_metadata(out_bag.string());
  auto first_storage = out_bag / metadata.relative_file_paths[0];

  EXPECT_EQ(first_storage.extension().string(), ".zstd");
  EXPECT_TRUE(fs::exists(first_storage));
  EXPECT_TRUE(fs::is_regular_file(first_storage));
}

TEST_P(TestRewrite, test_cut_single_topic_with_start_and_end_time) {
  use_input_a();

  // Scan input bag for a single topic (a_empty) to compute expected counts in a time window
  rosbag2_storage::StorageOptions scan_opts = input_bags_[0];
  auto scan_reader = rosbag2_transport::ReaderWriterFactory::make_reader(scan_opts);
  scan_reader->open(scan_opts);

  std::vector<rcutils_time_point_value_t> timestamps;
  while (scan_reader->has_next()) {
    auto msg = scan_reader->read_next();
    if (msg && msg->topic_name == "a_empty") {
      timestamps.push_back(msg->recv_timestamp);
    }
  }
  ASSERT_THAT(timestamps, SizeIs(Ge(4u)));

  // Choose a mid-window based on observed timestamps
  // Window: [input_start_time, input_end_time],
  // expecting messages with recv_timestamp >= input_start_time and <= input_end_time
  auto input_start_time = timestamps[timestamps.size() / 4];
  auto input_end_time = timestamps[(timestamps.size() * 3) / 4];

  size_t expected_msg_count = 0;
  for (auto current_timestamp : timestamps) {
    // Note: In the test bag there are multiple messages with the same timestamp as the start and
    // end time. We will count all of them to include all messages in the [t_start, t_end] interval.
    if (current_timestamp >= input_start_time && current_timestamp <= input_end_time) {
      expected_msg_count++;
    }
  }

  // Prepare output with topic filter to just a_empty
  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "cut_window").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = false;
  record_options.topics = {"a_empty"};
  output_bags_.emplace_back(output_storage, record_options);

  // Apply cutting limits on input reader
  input_bags_[0].start_time_ns = input_start_time;
  input_bags_[0].end_time_ns = input_end_time;

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output_storage);
  reader->open(output_storage);
  const auto metadata = reader->get_metadata();
  EXPECT_THAT(metadata.topics_with_message_count, SizeIs(1));
  EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.name, "a_empty");

  EXPECT_EQ(metadata.message_count, expected_msg_count) <<
    "input_start_time: " << input_start_time << ", input_end_time: " << input_end_time;

  // If the count is wrong, print out timestamps for debugging
  if (metadata.message_count != expected_msg_count) {
    // Print first and last timestamp for debugging
    auto first_msg = reader->read_next();
    auto last_msg = first_msg;
    size_t msg_count = 1;
    std::cerr << "Output msg timestamp: " << first_msg->recv_timestamp <<
      " msg_count: " << msg_count << std::endl;
    while (reader->has_next()) {
      msg_count++;
      last_msg = reader->read_next();
      std::cerr << "Output msg timestamp: " << last_msg->recv_timestamp <<
        " msg_count: " << msg_count << std::endl;
    }
    std::cerr << "First msg timestamp: " << first_msg->recv_timestamp << std::endl;
    std::cerr << "Last msg timestamp: " << last_msg->recv_timestamp << std::endl;
    std::cerr << "Message count: " << msg_count << std::endl;
    // Print original timestamps for debugging
    std::cerr << std::endl << "Original timestamps:" << std::endl;
    msg_count = 0;
    for (auto ts : timestamps) {
      msg_count++;
      std::cerr << "Original msg timestamp: " << ts << " msg_count: " << msg_count << std::endl;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  ParametrizedRewriteTests,
  TestRewrite,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);

namespace
{
/// Payload mimicking a protobuf encoded foxglove.CompressedVideo message. The exact bytes do
/// not matter to the rewrite, which has to pass them through unmodified.
std::vector<uint8_t> make_fake_protobuf_payload(uint8_t index)
{
  std::vector<uint8_t> payload = {0x12, 0x04, 'c', 'a', 'm', '0', 0x1a, 0x10};
  payload.insert(payload.end(), 16, index);
  const std::vector<uint8_t> format_field = {0x22, 0x04, 'h', '2', '6', '4'};
  payload.insert(payload.end(), format_field.begin(), format_field.end());
  return payload;
}
}  // namespace

TEST_F(TemporaryDirectoryFixture, test_rewrite_bag_with_mixed_serialization_formats) {
  const auto input_bag_path = fs::path(temporary_dir_path_) / "mixed_input";
  const std::string cdr_topic = "/chatter";
  const std::string protobuf_topic = "/camera/video_compressed";
  constexpr size_t kNumMessagesPerTopic = 5;

  // Write an input bag with one topic in the local rmw serialization format and one protobuf
  // encoded topic, mimicking recordings made by non-ROS tools.
  {
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions options;
    options.uri = input_bag_path.string();
    options.storage_id = "mcap";
    writer.open(options);

    rosbag2_storage::TopicMetadata protobuf_topic_metadata;
    protobuf_topic_metadata.name = protobuf_topic;
    protobuf_topic_metadata.type = "foxglove.CompressedVideo";
    protobuf_topic_metadata.serialization_format = "protobuf";
    writer.create_topic(protobuf_topic_metadata);

    test_msgs::msg::BasicTypes msg;
    for (size_t i = 0; i < kNumMessagesPerTopic; i++) {
      const int64_t stamp_ns = 10000000 * static_cast<int64_t>(i);
      msg.int32_value = static_cast<int32_t>(i);
      writer.write(msg, cdr_topic, rclcpp::Time(stamp_ns));

      const auto payload = make_fake_protobuf_payload(static_cast<uint8_t>(i));
      auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      message->topic_name = protobuf_topic;
      message->recv_timestamp = stamp_ns + 1;
      message->send_timestamp = stamp_ns + 1;
      message->serialized_data =
        rosbag2_storage::make_serialized_message(payload.data(), payload.size());
      writer.write(message);
    }
  }

  rosbag2_storage::StorageOptions input_storage;
  input_storage.uri = input_bag_path.string();
  input_storage.storage_id = "mcap";

  rosbag2_storage::StorageOptions all_topics_storage;
  all_topics_storage.uri = (fs::path(temporary_dir_path_) / "mixed_output").string();
  all_topics_storage.storage_id = "mcap";
  rosbag2_transport::RecordOptions all_topics_record;
  all_topics_record.all_topics = true;

  rosbag2_storage::StorageOptions cdr_only_storage;
  cdr_only_storage.uri = (fs::path(temporary_dir_path_) / "cdr_only_output").string();
  cdr_only_storage.storage_id = "mcap";
  rosbag2_transport::RecordOptions cdr_only_record;
  cdr_only_record.all_topics = false;
  cdr_only_record.topics = {cdr_topic};

  rosbag2_transport::bag_rewrite(
    {input_storage},
    {{all_topics_storage, all_topics_record}, {cdr_only_storage, cdr_only_record}});

  // The unfiltered output shall contain both topics, with their serialization formats
  // preserved and the protobuf messages passed through byte for byte.
  {
    auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(all_topics_storage);
    reader->open(all_topics_storage);

    std::unordered_map<std::string, std::string> serialization_formats;
    for (const auto & topic : reader->get_all_topics_and_types()) {
      serialization_formats[topic.name] = topic.serialization_format;
    }
    EXPECT_THAT(serialization_formats, SizeIs(2));
    EXPECT_EQ(serialization_formats[protobuf_topic], "protobuf");
    EXPECT_FALSE(serialization_formats[cdr_topic].empty());
    EXPECT_NE(serialization_formats[cdr_topic], "protobuf");

    std::unordered_map<std::string, size_t> counts;
    while (reader->has_next()) {
      const auto message = reader->read_next();
      if (message->topic_name == protobuf_topic) {
        const auto expected_payload =
          make_fake_protobuf_payload(static_cast<uint8_t>(counts[protobuf_topic]));
        ASSERT_NE(nullptr, message->serialized_data);
        ASSERT_EQ(expected_payload.size(), message->serialized_data->buffer_length);
        EXPECT_EQ(
          0,
          memcmp(
            expected_payload.data(), message->serialized_data->buffer, expected_payload.size()));
      }
      counts[message->topic_name]++;
    }
    EXPECT_EQ(counts[cdr_topic], kNumMessagesPerTopic);
    EXPECT_EQ(counts[protobuf_topic], kNumMessagesPerTopic);
  }

  // The filtered output shall only contain the topic in the local rmw serialization format.
  {
    auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(cdr_only_storage);
    reader->open(cdr_only_storage);
    const auto metadata = reader->get_metadata();
    ASSERT_THAT(metadata.topics_with_message_count, SizeIs(1));
    EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.name, cdr_topic);
    EXPECT_EQ(metadata.message_count, kNumMessagesPerTopic);
  }
}
