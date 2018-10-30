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
#include <tuple>
#include <utility>
#include <vector>

#include "rcutils/snprintf.h"

#include "rosbag2_storage/filesystem_helper.hpp"
#include "storage_test_fixture.hpp"

using namespace ::testing;  // NOLINT

namespace rosbag2_storage
{

bool operator==(const TopicWithType & lhs, const TopicWithType & rhs)
{
  return lhs.name == rhs.name && lhs.type == rhs.type;
}

bool operator!=(const TopicWithType & lhs, const TopicWithType & rhs)
{
  return !(lhs == rhs);
}

bool operator==(const TopicMetadata & lhs, const TopicMetadata & rhs)
{
  return lhs.topic_with_type == rhs.topic_with_type &&
         lhs.message_count == rhs.message_count;
}

bool operator!=(const TopicMetadata & lhs, const TopicMetadata & rhs)
{
  return !(lhs == rhs);
}

}  // namespace rosbag2_storage

TEST_F(StorageTestFixture, string_messages_are_written_and_read_to_and_from_sqlite3_storage) {
  std::vector<std::string> string_messages = {"first message", "second message", "third message"};
  std::vector<std::string> topics = {"topic1", "topic2", "topic3"};
  std::vector<std::tuple<std::string, int64_t, std::string, std::string>> messages =
  {std::make_tuple(string_messages[0], 1, topics[0], "type1"),
    std::make_tuple(string_messages[1], 2, topics[1], "type2"),
    std::make_tuple(string_messages[2], 3, topics[2], "type3")};

  write_messages_to_sqlite(messages);
  auto read_messages = read_all_messages_from_sqlite();

  ASSERT_THAT(read_messages, SizeIs(3));
  for (size_t i = 0; i < 3; i++) {
    EXPECT_THAT(deserialize_message(read_messages[i]->serialized_data), Eq(string_messages[i]));
    EXPECT_THAT(read_messages[i]->time_stamp, Eq(std::get<1>(messages[i])));
    EXPECT_THAT(read_messages[i]->topic_name, Eq(topics[i]));
  }
}

TEST_F(StorageTestFixture, has_next_return_false_if_there_are_no_more_messages) {
  std::vector<std::tuple<std::string, int64_t, std::string, std::string>> string_messages =
  {std::make_tuple("first message", 1, "", ""), std::make_tuple("second message", 2, "", "")};

  write_messages_to_sqlite(string_messages);
  std::unique_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> readable_storage =
    std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
  readable_storage->open(temporary_dir_path_);

  EXPECT_TRUE(readable_storage->has_next());
  readable_storage->read_next();
  EXPECT_TRUE(readable_storage->has_next());
  readable_storage->read_next();
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(StorageTestFixture, get_next_returns_messages_in_timestamp_order) {
  std::vector<std::tuple<std::string, int64_t, std::string, std::string>> string_messages =
  {std::make_tuple("first_message", 6, "", ""), std::make_tuple("second_message", 2, "", "")};

  write_messages_to_sqlite(string_messages);
  std::unique_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> readable_storage =
    std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
  readable_storage->open(temporary_dir_path_);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->time_stamp, Eq(2));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->time_stamp, Eq(6));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(StorageTestFixture, get_all_topics_and_types_returns_the_correct_vector) {
  std::unique_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> writable_storage =
    std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
  writable_storage->open(temporary_dir_path_);
  writable_storage->create_topic({"topic1", "type1"});
  writable_storage->create_topic({"topic2", "type2"});
  metadata_io_.write_metadata(temporary_dir_path_, writable_storage->get_metadata());
  writable_storage.reset();

  auto readable_storage = std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
  readable_storage->open(
    temporary_dir_path_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
  auto topics_and_types = readable_storage->get_all_topics_and_types();

  EXPECT_THAT(topics_and_types, ElementsAreArray({
    rosbag2_storage::TopicWithType{"topic1", "type1"},
    rosbag2_storage::TopicWithType{"topic2", "type2"}
  }));
}

TEST_F(StorageTestFixture, get_metadata_returns_correct_struct) {
  std::vector<std::string> string_messages = {"first message", "second message", "third message"};
  std::vector<std::string> topics = {"topic1", "topic2"};
  std::vector<std::tuple<std::string, int64_t, std::string, std::string>> messages =
  {std::make_tuple(string_messages[0], static_cast<int64_t>(1e9), topics[0], "type1"),
    std::make_tuple(string_messages[1], static_cast<int64_t>(2e9), topics[0], "type1"),
    std::make_tuple(string_messages[2], static_cast<int64_t>(3e9), topics[1], "type2")};

  write_messages_to_sqlite(messages);

  auto readable_storage = std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
  readable_storage->open(
    temporary_dir_path_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
  auto metadata = readable_storage->get_metadata();

  EXPECT_THAT(metadata.storage_identifier, Eq("sqlite3"));
  EXPECT_THAT(metadata.serialization_format, Eq("cdr"));
  EXPECT_THAT(metadata.relative_file_paths, ElementsAreArray({
    rosbag2_storage::FilesystemHelper::get_folder_name(temporary_dir_path_) + ".db3"
  }));
  EXPECT_THAT(metadata.topics_with_message_count, ElementsAreArray({
    rosbag2_storage::TopicMetadata{rosbag2_storage::TopicWithType{"topic1", "type1"}, 2u},
    rosbag2_storage::TopicMetadata{rosbag2_storage::TopicWithType{"topic2", "type2"}, 1u}
  }));
  EXPECT_THAT(metadata.message_count, Eq(3u));
  EXPECT_THAT(metadata.starting_time, Eq(
      std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::seconds(1))
  ));
  EXPECT_THAT(metadata.duration, Eq(std::chrono::seconds(2)));
}

TEST_F(StorageTestFixture, get_metadata_returns_correct_struct_if_no_messages) {
  write_messages_to_sqlite({});

  auto readable_storage = std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
  readable_storage->open(
    temporary_dir_path_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
  auto metadata = readable_storage->get_metadata();

  EXPECT_THAT(metadata.storage_identifier, Eq("sqlite3"));
  EXPECT_THAT(metadata.serialization_format, Eq("cdr"));
  EXPECT_THAT(metadata.relative_file_paths, ElementsAreArray({
    rosbag2_storage::FilesystemHelper::get_folder_name(temporary_dir_path_) + ".db3"
  }));
  EXPECT_THAT(metadata.topics_with_message_count, IsEmpty());
  EXPECT_THAT(metadata.message_count, Eq(0u));
  EXPECT_THAT(metadata.starting_time, Eq(
      std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::seconds(0))
  ));
  EXPECT_THAT(metadata.duration, Eq(std::chrono::seconds(0)));
}
