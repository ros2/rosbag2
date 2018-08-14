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
#include <utility>
#include <vector>

#include "rcutils/snprintf.h"

#include "storage_test_fixture.hpp"

using namespace ::testing;  // NOLINT

TEST_F(StorageTestFixture, string_messages_are_written_and_read_to_and_from_sqlite3_storage) {
  std::vector<std::pair<std::string, int64_t>> string_messages =
  {std::make_pair("first message", 1),
    std::make_pair("second message", 2),
    std::make_pair("third message", 3)};

  write_messages_to_sqlite(string_messages);
  auto read_messages = read_all_messages_from_sqlite();

  ASSERT_THAT(read_messages, SizeIs(3));
  EXPECT_THAT(deserialize_message(read_messages[0]), Eq(string_messages[0].first));
  EXPECT_THAT(deserialize_message(read_messages[1]), Eq(string_messages[1].first));
  EXPECT_THAT(deserialize_message(read_messages[2]), Eq(string_messages[2].first));
}

TEST_F(StorageTestFixture, message_roundtrip_with_arbitrary_char_array_works_correctly) {
  std::string message = "test_message";
  size_t message_size = strlen(message.c_str()) + 1;
  char * test_message = new char[message_size];
  memcpy(test_message, message.c_str(), message_size);

  std::unique_ptr<rosbag2_storage::ReadWriteStorage> read_write_storage =
    std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
  read_write_storage->open(database_name_);
  read_write_storage->create_topic();

  rosbag2_storage::SerializedBagMessage test;
  test.serialized_data = std::make_shared<rcutils_char_array_t>();
  test.serialized_data->buffer = test_message;
  test.serialized_data->buffer_length = message_size;
  test.serialized_data->buffer_capacity = message_size;

  read_write_storage->write(test);

  auto read_message = read_write_storage->read_next();
  EXPECT_THAT(strcmp(read_message.serialized_data->buffer, test_message), Eq(0));

  delete[] test_message;
}


TEST_F(StorageTestFixture, has_next_return_false_if_there_are_no_more_messages) {
  std::vector<std::pair<std::string, int64_t>> string_messages =
  {std::make_pair("first message", 1), std::make_pair("second message", 2)};

  write_messages_to_sqlite(string_messages);
  std::unique_ptr<rosbag2_storage::ReadableStorage> readable_storage =
    std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
  readable_storage->open_readonly(database_name_);

  EXPECT_TRUE(readable_storage->has_next());
  readable_storage->read_next();
  EXPECT_TRUE(readable_storage->has_next());
  readable_storage->read_next();
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(StorageTestFixture, write_stamped_char_array_writes_correct_time_stamp) {
  std::vector<std::pair<std::string, int64_t>> string_messages =
  {std::make_pair("first message", 1), std::make_pair("second message", 2)};

  write_messages_to_sqlite(string_messages);
  std::unique_ptr<rosbag2_storage::ReadableStorage> readable_storage =
    std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
  readable_storage->open_readonly(database_name_);

  auto read_message = readable_storage->read_next();
  EXPECT_THAT(read_message.time_stamp, Eq(1));
  read_message = readable_storage->read_next();
  EXPECT_THAT(read_message.time_stamp, Eq(2));
}
