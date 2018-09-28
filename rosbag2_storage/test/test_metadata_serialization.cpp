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
#include <memory>
#include <string>

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#endif

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "temporary_directory_fixture.hpp"

using namespace ::testing;  // NOLINT

class MetadataFixture : public TemporaryDirectoryFixture
{
public:
  MetadataFixture()
  : metadata_io_(std::make_shared<rosbag2_storage::MetadataIO>(
        temporary_dir_path_ + separator() + "test"))  // TODO(botteroa-si): as soon as the uri is
    // the path to the bag directory the ony parameter left is temporary_dir_path.
  {}

  // TODO(greimela): Move to common place (is also used in test fixture etc.)
  std::string separator()
  {
#ifdef _WIN32
    return "\\";
#else
    return "/";
#endif
  }

  std::shared_ptr<rosbag2_storage::MetadataIO> metadata_io_;
};

TEST_F(MetadataFixture, test_writing_and_reading_yaml)
{
  rosbag2_storage::BagMetadata metadata{};
  metadata.storage_identifier = "sqlite3";
  metadata.encoding = "cdr";
  metadata.relative_file_paths.emplace_back("some_relative_path");
  metadata.relative_file_paths.emplace_back("some_other_relative_path");
  metadata.combined_bag_size = 10;
  metadata.duration = std::chrono::nanoseconds(100);
  metadata.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(1000000));
  metadata.message_count = 50;
  metadata.topics_with_message_count.push_back({{"topic1", "type1"}, 100});
  metadata.topics_with_message_count.push_back({{"topic2", "type2"}, 200});

  metadata_io_->write_metadata(metadata);
  auto read_metadata = metadata_io_->read_metadata();

  EXPECT_THAT(read_metadata.storage_identifier, Eq(metadata.storage_identifier));
  EXPECT_THAT(read_metadata.encoding, Eq(metadata.encoding));
  EXPECT_THAT(read_metadata.relative_file_paths, Eq(metadata.relative_file_paths));
  EXPECT_THAT(read_metadata.combined_bag_size, Eq(metadata.combined_bag_size));
  EXPECT_THAT(read_metadata.duration, Eq(metadata.duration));
  EXPECT_THAT(read_metadata.starting_time, Eq(metadata.starting_time));
  EXPECT_THAT(read_metadata.message_count, Eq(metadata.message_count));
  EXPECT_THAT(read_metadata.topics_with_message_count,
    SizeIs(metadata.topics_with_message_count.size()));
  auto actual_first_topic = read_metadata.topics_with_message_count[0];
  auto expected_first_topic = read_metadata.topics_with_message_count[0];
  EXPECT_THAT(actual_first_topic.topic_with_type.name,
    Eq(expected_first_topic.topic_with_type.name));
  EXPECT_THAT(actual_first_topic.topic_with_type.type,
    Eq(expected_first_topic.topic_with_type.type));
  EXPECT_THAT(actual_first_topic.message_count, Eq(expected_first_topic.message_count));
  auto actual_second_topic = read_metadata.topics_with_message_count[1];
  auto expected_second_topic = read_metadata.topics_with_message_count[1];
  EXPECT_THAT(actual_second_topic.topic_with_type.name,
    Eq(expected_second_topic.topic_with_type.name));
  EXPECT_THAT(actual_second_topic.topic_with_type.type,
    Eq(expected_second_topic.topic_with_type.type));
  EXPECT_THAT(actual_second_topic.message_count, Eq(expected_second_topic.message_count));
}
