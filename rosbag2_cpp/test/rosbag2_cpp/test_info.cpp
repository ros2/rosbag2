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

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/default_storage_id.hpp"
#include "rosbag2_storage/metadata_io.hpp"

#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"

#include "test_msgs/msg/basic_types.hpp"

using namespace ::testing;  // NOLINT
using rosbag2_test_common::ParametrizedTemporaryDirectoryFixture;

TEST_P(ParametrizedTemporaryDirectoryFixture, read_metadata_supports_version_2) {
  const auto expected_storage_id = GetParam();
  const std::string bagfile = "rosbag2_bagfile_information:\n"
    "  version: 2\n"
    "  storage_identifier: " + expected_storage_id + "\n"
    "  relative_file_paths:\n"
    "    - some_relative_path\n"
    "    - some_other_relative_path\n"
    "  duration:\n"
    "    nanoseconds: 100\n"
    "  starting_time:\n"
    "    nanoseconds_since_epoch: 1000000\n"
    "  message_count: 50\n"
    "  topics_with_message_count:\n"
    "    - topic_metadata:\n"
    "        name: topic1\n"
    "        type: type1\n"
    "        serialization_format: rmw1\n"
    "      message_count: 100\n"
    "    - topic_metadata:\n"
    "        name: topic2\n"
    "        type: type2\n"
    "        serialization_format: rmw2\n"
    "      message_count: 200\n";

  {
    std::ofstream fout {
      (rcpputils::fs::path(temporary_dir_path_) / rosbag2_storage::MetadataIo::metadata_filename)
      .string()};
    fout << bagfile;
  }

  rosbag2_cpp::Info info;
  const auto metadata = info.read_metadata(temporary_dir_path_);

  EXPECT_EQ(metadata.version, 2);
  EXPECT_EQ(metadata.storage_identifier, expected_storage_id);

  const auto expected_paths =
    std::vector<std::string>{"some_relative_path", "some_other_relative_path"};
  EXPECT_EQ(metadata.relative_file_paths, expected_paths);
  EXPECT_EQ(metadata.duration, std::chrono::nanoseconds{100});
  EXPECT_EQ(
    metadata.starting_time,
    std::chrono::time_point<std::chrono::high_resolution_clock>{std::chrono::nanoseconds{1000000}});
  EXPECT_EQ(metadata.message_count, 50u);
  EXPECT_EQ(metadata.topics_with_message_count.size(), 2u);

  {
    const auto actual_first_topic = metadata.topics_with_message_count[0];
    const auto expected_first_topic =
      rosbag2_storage::TopicInformation{{"topic1", "type1", "rmw1", ""}, 100};

    EXPECT_EQ(actual_first_topic.topic_metadata, expected_first_topic.topic_metadata);
    EXPECT_EQ(actual_first_topic.message_count, expected_first_topic.message_count);
  }

  {
    const auto actual_second_topic = metadata.topics_with_message_count[1];
    const auto expected_second_topic =
      rosbag2_storage::TopicInformation{{"topic2", "type2", "rmw2", ""}, 200};

    EXPECT_EQ(actual_second_topic.topic_metadata, expected_second_topic.topic_metadata);
    EXPECT_EQ(actual_second_topic.message_count, expected_second_topic.message_count);
  }
}

TEST_P(
  ParametrizedTemporaryDirectoryFixture,
  read_metadata_makes_appropriate_call_to_metadata_io_method) {
  const auto expected_storage_id = GetParam();
  std::string bagfile(
    "rosbag2_bagfile_information:\n"
    "  version: 3\n"
    "  storage_identifier: " + expected_storage_id + "\n"
    "  relative_file_paths:\n"
    "    - some_relative_path\n"
    "    - some_other_relative_path\n"
    "  combined_bag_size: 10\n"
    "  duration:\n"
    "    nanoseconds: 100\n"
    "  starting_time:\n"
    "    nanoseconds_since_epoch: 1000000\n"
    "  message_count: 50\n"
    "  topics_with_message_count:\n"
    "    - topic_metadata:\n"
    "        name: topic1\n"
    "        type: type1\n"
    "        serialization_format: rmw1\n"
    "      message_count: 100\n"
    "    - topic_metadata:\n"
    "        name: topic2\n"
    "        type: type2\n"
    "        serialization_format: rmw2\n"
    "      message_count: 200\n"
    "  compression_format: \"zstd\"\n"
    "  compression_mode: \"FILE\"\n");

  std::ofstream fout {
    (rcpputils::fs::path(temporary_dir_path_) / rosbag2_storage::MetadataIo::metadata_filename)
    .string()};
  fout << bagfile;
  fout.close();

  rosbag2_cpp::Info info;
  auto read_metadata = info.read_metadata(temporary_dir_path_);

  EXPECT_EQ(read_metadata.storage_identifier, expected_storage_id);
  EXPECT_THAT(
    read_metadata.relative_file_paths,
    Eq(std::vector<std::string>({"some_relative_path", "some_other_relative_path"})));
  EXPECT_THAT(read_metadata.duration, Eq(std::chrono::nanoseconds(100)));
  EXPECT_THAT(
    read_metadata.starting_time,
    Eq(
      std::chrono::time_point<std::chrono::high_resolution_clock>(
        std::chrono::nanoseconds(1000000))));
  EXPECT_THAT(read_metadata.message_count, Eq(50u));
  EXPECT_THAT(read_metadata.topics_with_message_count, SizeIs(2u));
  auto actual_first_topic = read_metadata.topics_with_message_count[0];
  rosbag2_storage::TopicInformation expected_first_topic = {{"topic1", "type1", "rmw1", ""}, 100};
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
  rosbag2_storage::TopicInformation expected_second_topic = {{"topic2", "type2", "rmw2", ""}, 200};
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

  EXPECT_EQ(read_metadata.compression_format, "zstd");
  EXPECT_EQ(read_metadata.compression_mode, "FILE");
}

TEST_P(ParametrizedTemporaryDirectoryFixture, info_for_standalone_bagfile) {
  const auto storage_id = GetParam();
  const auto bag_path = rcpputils::fs::path(temporary_dir_path_) / "bag";
  {
    // Create an empty bag with default storage
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.storage_id = storage_id;
    storage_options.uri = bag_path.string();
    writer.open(storage_options);
    test_msgs::msg::BasicTypes msg;
    writer.write(msg, "testtopic", rclcpp::Time{});
  }

  std::string first_storage_file_path;
  {
    rosbag2_storage::MetadataIo metadata_io;
    auto metadata = metadata_io.read_metadata(bag_path.string());
    first_storage_file_path = (bag_path / metadata.relative_file_paths[0]).string();
  }
  rosbag2_cpp::Info info;
  rosbag2_storage::BagMetadata metadata;
  EXPECT_NO_THROW(
    metadata = info.read_metadata(first_storage_file_path)
  );
  EXPECT_THAT(metadata.topics_with_message_count, SizeIs(1));
}

INSTANTIATE_TEST_SUITE_P(
  RosbagInfoTests,
  ParametrizedTemporaryDirectoryFixture,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
