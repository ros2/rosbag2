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
#include "rcpputils/filesystem_helper.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "yaml-cpp/yaml.h"

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
  metadata.topics_with_message_count.push_back({{"topic1", "type1", "rmw1", "qos1"}, 100});
  metadata.topics_with_message_count.push_back({{"topic2", "type2", "rmw2", "qos2"}, 200});

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
  const std::string offered_qos_profiles = "qos_profile_string_data";
  const size_t message_count = 100;  // arbitrary value

  BagMetadata metadata{};
  metadata.version = 3;
  metadata.topics_with_message_count.push_back(
    {{"topic", "type", "rmw", offered_qos_profiles}, message_count});
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
  const std::string offered_qos_profiles = "qos_profile_string_data";
  const size_t message_count = 100;  // arbitrary value

  BagMetadata metadata{};
  metadata.version = 4;
  metadata.topics_with_message_count.push_back(
    {{"topic", "type", "rmw", offered_qos_profiles}, message_count});
  metadata_io_->write_metadata(temporary_dir_path_, metadata);
  auto read_metadata = metadata_io_->read_metadata(temporary_dir_path_);
  ASSERT_THAT(
    read_metadata.topics_with_message_count,
    SizeIs(metadata.topics_with_message_count.size()));
  auto actual_first_topic = read_metadata.topics_with_message_count[0];
  EXPECT_THAT(actual_first_topic.topic_metadata.offered_qos_profiles, Eq(offered_qos_profiles));
}

TEST_F(MetadataFixture, metadata_reads_v9_offered_qos_profiles_as_sequence)
{
  // Newer ROS 2 distributions write "offered_qos_profiles" as a YAML sequence instead of a
  // YAML-encoded string, with policies stored by name. Bags in that format must be readable.
  const std::string v9_metadata =
    "rosbag2_bagfile_information:\n"
    "  version: 9\n"
    "  storage_identifier: mcap\n"
    "  duration:\n"
    "    nanoseconds: 100\n"
    "  starting_time:\n"
    "    nanoseconds_since_epoch: 1000000\n"
    "  message_count: 14\n"
    "  topics_with_message_count:\n"
    "    - topic_metadata:\n"
    "        name: /latched\n"
    "        type: std_msgs/msg/Int32\n"
    "        serialization_format: cdr\n"
    "        offered_qos_profiles:\n"
    "          - history: unknown\n"
    "            depth: 0\n"
    "            reliability: reliable\n"
    "            durability: transient_local\n"
    "            deadline:\n"
    "              sec: 9223372036\n"
    "              nsec: 854775807\n"
    "            lifespan:\n"
    "              sec: 9223372036\n"
    "              nsec: 854775807\n"
    "            liveliness: automatic\n"
    "            liveliness_lease_duration:\n"
    "              sec: 9223372036\n"
    "              nsec: 854775807\n"
    "            avoid_ros_namespace_conventions: false\n"
    "        type_description_hash: RIHS01_b6578ded3c58c626cfe8d1a6fb6e04f706f97e9f03d2727c9ff4e7\n"
    "      message_count: 14\n"
    "  compression_format: \"\"\n"
    "  compression_mode: \"\"\n"
    "  relative_file_paths:\n"
    "    - bag_0.mcap\n"
    "  files:\n"
    "    - path: bag_0.mcap\n"
    "      starting_time:\n"
    "        nanoseconds_since_epoch: 1000000\n"
    "      duration:\n"
    "        nanoseconds: 100\n"
    "      message_count: 14\n"
    "  custom_data: ~\n"
    "  ros_distro: jazzy\n";
  {
    std::ofstream fout(
      (rcpputils::fs::path(temporary_dir_path_) / MetadataIo::metadata_filename).string());
    fout << v9_metadata;
  }

  auto read_metadata = metadata_io_->read_metadata(temporary_dir_path_);
  EXPECT_THAT(read_metadata.version, Eq(9));
  ASSERT_THAT(read_metadata.topics_with_message_count, SizeIs(1u));
  const auto & topic = read_metadata.topics_with_message_count[0].topic_metadata;
  EXPECT_THAT(topic.name, Eq("/latched"));

  // The QoS profiles are kept as a YAML-encoded string, as with previous metadata versions
  const YAML::Node qos_node = YAML::Load(topic.offered_qos_profiles);
  ASSERT_TRUE(qos_node.IsSequence());
  ASSERT_THAT(qos_node.size(), Eq(1u));
  EXPECT_THAT(qos_node[0]["reliability"].as<std::string>(), Eq("reliable"));
  EXPECT_THAT(qos_node[0]["durability"].as<std::string>(), Eq("transient_local"));
}
