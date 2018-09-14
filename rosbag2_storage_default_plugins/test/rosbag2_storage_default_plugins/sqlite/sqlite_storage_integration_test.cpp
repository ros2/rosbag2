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

#include "storage_test_fixture.hpp"

using namespace ::testing;  // NOLINT

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
  readable_storage->open(database_name_);

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
  readable_storage->open(database_name_);

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
  writable_storage->open(database_name_);
  writable_storage->create_topic("topic1", "type1");
  writable_storage->create_topic("topic2", "type2");
  writable_storage.reset();

  auto readable_storage = std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
  readable_storage->open(database_name_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
  auto topics_and_types = readable_storage->get_all_topics_and_types();

  EXPECT_THAT(topics_and_types, SizeIs(2));
  EXPECT_THAT(topics_and_types[0].name, Eq("topic1"));
  EXPECT_THAT(topics_and_types[0].type, Eq("type1"));
  EXPECT_THAT(topics_and_types[1].name, Eq("topic2"));
  EXPECT_THAT(topics_and_types[1].type, Eq("type2"));
}
