// Copyright 2022, Foxglove Technologies. All rights reserved.
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

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#ifdef ROSBAG2_STORAGE_MCAP_HAS_STORAGE_OPTIONS
  #include "rosbag2_storage/storage_options.hpp"
using StorageOptions = rosbag2_storage::StorageOptions;
#else
  #include "rosbag2_cpp/storage_options.hpp"
using StorageOptions = rosbag2_cpp::StorageOptions;
#endif
#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "std_msgs/msg/string.hpp"

#include <gmock/gmock.h>

#include <memory>
#include <string>

using namespace ::testing;  // NOLINT
using TemporaryDirectoryFixture = rosbag2_test_common::TemporaryDirectoryFixture;

TEST_F(TemporaryDirectoryFixture, can_write_and_read_basic_mcap_file)
{
  auto uri = rcpputils::fs::path(temporary_dir_path_) / "bag";
  auto expected_bag = uri / "bag_0.mcap";
  const int64_t timestamp_nanos = 100;  // arbitrary value
  rcutils_time_point_value_t time_stamp{timestamp_nanos};
  const std::string topic_name = "test_topic";
  const std::string message_data = "Test Message 1";
  const std::string storage_id = "mcap";
  // COMPATIBILITY(foxy)
  // using verbose APIs for Foxy compatibility which did not yet provide plain-message API
  rclcpp::Serialization<std_msgs::msg::String> serialization;

  {
    StorageOptions options;
    options.uri = uri.string();
    options.storage_id = storage_id;
    rosbag2_storage::TopicMetadata topic_metadata;
    topic_metadata.name = topic_name;
    topic_metadata.type = "std_msgs/msg/String";

    std_msgs::msg::String msg;
    msg.data = message_data;

    rosbag2_cpp::Writer writer{std::make_unique<rosbag2_cpp::writers::SequentialWriter>()};
    writer.open(options, rosbag2_cpp::ConverterOptions{});
    writer.create_topic(topic_metadata);

    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    serialization.serialize_message(&msg, serialized_msg.get());

    // This is really kludgy, it's due to a mismatch between types in the rclcpp serialization API
    // and the historical Foxy serialized APIs. Prevents the hacked shared ptr from deleting the
    // data that `serialized_bag_msg` should reasonably expect to continue existing.
    // For this example it wouldn't matter, but in case anybody extends this test, it's for safety.
    auto serialized_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    serialized_bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      const_cast<rcutils_uint8_array_t *>(&serialized_msg->get_rcl_serialized_message()),
      [](rcutils_uint8_array_t * /* data */) {});
    serialized_bag_msg->time_stamp = time_stamp;
    serialized_bag_msg->topic_name = topic_name;
    writer.write(serialized_bag_msg);
    EXPECT_TRUE(expected_bag.is_regular_file());
  }
  {
    StorageOptions options;
    options.uri = expected_bag.string();
    options.storage_id = storage_id;

    rosbag2_cpp::Reader reader{std::make_unique<rosbag2_cpp::readers::SequentialReader>()};
    reader.open(options, rosbag2_cpp::ConverterOptions{});
    EXPECT_TRUE(reader.has_next());

    std_msgs::msg::String msg;
    auto serialized_bag_msg = reader.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_bag_msg->serialized_data);
    serialization.deserialize_message(&extracted_serialized_msg, &msg);
    EXPECT_EQ(msg.data, message_data);
  }
}

#ifdef ROSBAG2_STORAGE_MCAP_HAS_STORAGE_OPTIONS
// This test disabled on Foxy since StorageOptions doesn't have storage_config_uri field on it
TEST_F(TemporaryDirectoryFixture, can_write_mcap_with_zstd_configured_from_yaml)
{
  auto uri = rcpputils::fs::path(temporary_dir_path_) / "bag";
  auto expected_bag = uri / "bag_0.mcap";
  const int64_t timestamp_nanos = 100;  // arbitrary value
  rcutils_time_point_value_t time_stamp{timestamp_nanos};
  const std::string topic_name = "test_topic";
  const std::string message_data = "Test Message 1";
  const std::string storage_id = "mcap";
  const std::string config_path = _TEST_RESOURCES_DIR_PATH;
  // COMPATIBILITY(foxy)
  // using verbose APIs for Foxy compatibility which did not yet provide plain-message API
  rclcpp::Serialization<std_msgs::msg::String> serialization;

  {
    StorageOptions options;
    options.uri = uri.string();
    options.storage_id = storage_id;
    options.storage_config_uri = config_path + "/mcap_writer_options_zstd.yaml";
    rosbag2_storage::TopicMetadata topic_metadata;
    topic_metadata.name = topic_name;
    topic_metadata.type = "std_msgs/msg/String";

    std_msgs::msg::String msg;
    msg.data = message_data;

    rosbag2_cpp::Writer writer{std::make_unique<rosbag2_cpp::writers::SequentialWriter>()};
    writer.open(options, rosbag2_cpp::ConverterOptions{});
    writer.create_topic(topic_metadata);

    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    serialization.serialize_message(&msg, serialized_msg.get());

    // This is really kludgy, it's due to a mismatch between types in the rclcpp serialization API
    // and the historical Foxy serialized APIs. Prevents the hacked shared ptr from deleting the
    // data that `serialized_bag_msg` should reasonably expect to continue existing.
    // For this example it wouldn't matter, but in case anybody extends this test, it's for safety.
    auto serialized_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    serialized_bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      const_cast<rcutils_uint8_array_t *>(&serialized_msg->get_rcl_serialized_message()),
      [](rcutils_uint8_array_t * /* data */) {});
    serialized_bag_msg->time_stamp = time_stamp;
    serialized_bag_msg->topic_name = topic_name;
    writer.write(serialized_bag_msg);
    writer.write(serialized_bag_msg);
    EXPECT_TRUE(expected_bag.is_regular_file());
  }
  {
    StorageOptions options;
    options.uri = expected_bag.string();
    options.storage_id = storage_id;

    rosbag2_cpp::Reader reader{std::make_unique<rosbag2_cpp::readers::SequentialReader>()};
    reader.open(options, rosbag2_cpp::ConverterOptions{});
    EXPECT_TRUE(reader.has_next());

    std_msgs::msg::String msg;
    auto serialized_bag_msg = reader.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_bag_msg->serialized_data);
    serialization.deserialize_message(&extracted_serialized_msg, &msg);
    EXPECT_EQ(msg.data, message_data);
  }
}
#endif  // #ifdef ROSBAG2_STORAGE_MCAP_HAS_STORAGE_OPTIONS
