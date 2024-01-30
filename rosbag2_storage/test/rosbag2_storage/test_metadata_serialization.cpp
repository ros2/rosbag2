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
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#endif

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/default_storage_id.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_storage;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class MetadataFixture : public TemporaryDirectoryFixture
{
public:
  MetadataFixture()
  : metadata_io_(std::make_shared<MetadataIo>()) {}

  std::shared_ptr<MetadataIo> metadata_io_;
};

TEST_F(MetadataFixture, test_writing_and_reading_yaml)
{
  BagMetadata metadata{};
  metadata.version = 1;
  metadata.storage_identifier = get_default_storage_id();
  metadata.relative_file_paths.emplace_back("some_relative_path");
  metadata.relative_file_paths.emplace_back("some_other_relative_path");
  metadata.duration = std::chrono::nanoseconds(100);
  metadata.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(1000000));
  metadata.message_count = 50;
  metadata.topics_with_message_count.push_back({{0u, "topic1", "type1", "rmw1", {}, ""}, 100});
  metadata.topics_with_message_count.push_back({{0u, "topic2", "type2", "rmw2", {}, ""}, 200});

  metadata_io_->write_metadata(temporary_dir_path_, metadata);
  auto read_metadata = metadata_io_->read_metadata(temporary_dir_path_);

  EXPECT_THAT(read_metadata.version, Eq(metadata.version));
  EXPECT_THAT(read_metadata.storage_identifier, Eq(metadata.storage_identifier));
  EXPECT_THAT(read_metadata.relative_file_paths, Eq(metadata.relative_file_paths));
  EXPECT_THAT(read_metadata.duration, Eq(metadata.duration));
  EXPECT_THAT(read_metadata.starting_time, Eq(metadata.starting_time));
  EXPECT_THAT(read_metadata.message_count, Eq(metadata.message_count));
  EXPECT_THAT(
    read_metadata.topics_with_message_count,
    SizeIs(metadata.topics_with_message_count.size()));
  auto actual_first_topic = read_metadata.topics_with_message_count[0];
  auto expected_first_topic = metadata.topics_with_message_count[0];
  EXPECT_THAT(
    actual_first_topic.topic_metadata.name,
    Eq(expected_first_topic.topic_metadata.name));
  EXPECT_THAT(
    actual_first_topic.topic_metadata.type,
    Eq(expected_first_topic.topic_metadata.type));
  EXPECT_THAT(
    actual_first_topic.topic_metadata.serialization_format,
    Eq(expected_first_topic.topic_metadata.serialization_format));
  EXPECT_THAT(actual_first_topic.message_count, Eq(expected_first_topic.message_count));
  auto actual_second_topic = read_metadata.topics_with_message_count[1];
  auto expected_second_topic = metadata.topics_with_message_count[1];
  EXPECT_THAT(
    actual_second_topic.topic_metadata.name,
    Eq(expected_second_topic.topic_metadata.name));
  EXPECT_THAT(
    actual_second_topic.topic_metadata.type,
    Eq(expected_second_topic.topic_metadata.type));
  EXPECT_THAT(
    actual_second_topic.topic_metadata.serialization_format,
    Eq(expected_second_topic.topic_metadata.serialization_format));
  EXPECT_THAT(actual_second_topic.message_count, Eq(expected_second_topic.message_count));
}

TEST_F(MetadataFixture, metadata_reads_v3_check_offered_qos_profiles_empty)
{
  // Make sure that when reading a v3 metadata, the deserialization
  // does not attempt to fill the offered_qos_profiles field
  std::vector<rclcpp::QoS> offered_qos_profiles;
  offered_qos_profiles.push_back(rclcpp::QoS(1));
  const size_t message_count = 100;  // arbitrary value

  BagMetadata metadata{};
  metadata.version = 3;
  metadata.topics_with_message_count.push_back(
    {{0u, "topic", "type", "rmw", offered_qos_profiles, ""}, message_count});
  metadata_io_->write_metadata(temporary_dir_path_, metadata);
  auto read_metadata = metadata_io_->read_metadata(temporary_dir_path_);
  ASSERT_THAT(
    read_metadata.topics_with_message_count,
    SizeIs(metadata.topics_with_message_count.size()));
  auto actual_first_topic = read_metadata.topics_with_message_count[0];
  EXPECT_TRUE(actual_first_topic.topic_metadata.offered_qos_profiles.empty());
}

TEST_F(MetadataFixture, metadata_reads_v4_fills_offered_qos_profiles)
{
  // Make sure when reading a v4 metadata that the deserialization fills "offered_qos_profiles"
  std::vector<rclcpp::QoS> offered_qos_profiles;
  offered_qos_profiles.push_back(rclcpp::QoS(1));
  const size_t message_count = 100;  // arbitrary value

  BagMetadata metadata{};
  metadata.version = 4;
  metadata.topics_with_message_count.push_back(
    {{0u, "topic", "type", "rmw", offered_qos_profiles, ""}, message_count});
  metadata_io_->write_metadata(temporary_dir_path_, metadata);
  auto read_metadata = metadata_io_->read_metadata(temporary_dir_path_);
  ASSERT_THAT(
    read_metadata.topics_with_message_count,
    SizeIs(metadata.topics_with_message_count.size()));
  auto actual_first_topic = read_metadata.topics_with_message_count[0];
  EXPECT_THAT(actual_first_topic.topic_metadata.offered_qos_profiles, Eq(offered_qos_profiles));
}

TEST_F(MetadataFixture, metadata_reads_v6_custom_data)
{
  BagMetadata metadata{};
  metadata.version = 6;
  metadata.custom_data["key1"] = "value1";
  metadata.custom_data["key2"] = "value2";

  metadata_io_->write_metadata(temporary_dir_path_, metadata);
  auto read_metadata = metadata_io_->read_metadata(temporary_dir_path_);

  EXPECT_THAT(read_metadata.custom_data, Eq(metadata.custom_data));
}

TEST_F(MetadataFixture, metadata_reads_v7_topic_type_hash)
{
  const std::string type_description_hash = "hash";

  BagMetadata metadata{};
  metadata.version = 7;
  metadata.topics_with_message_count.push_back(
    {{0u, "topic", "type", "rmw", {rclcpp::QoS(1)}, type_description_hash}, 1});

  metadata_io_->write_metadata(temporary_dir_path_, metadata);
  auto read_metadata = metadata_io_->read_metadata(temporary_dir_path_);

  ASSERT_THAT(
    read_metadata.topics_with_message_count,
    SizeIs(metadata.topics_with_message_count.size()));
  auto actual_first_topic = read_metadata.topics_with_message_count[0];
  EXPECT_THAT(actual_first_topic.topic_metadata.type_description_hash, Eq(type_description_hash));
}
