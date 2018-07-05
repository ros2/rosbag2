/*
 *  Copyright (c) 2018,  Bosch Software Innovations GmbH.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>

#include "../../../src/rosbag2/storage/sqlite/sqlite_storage.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT

class SqliteStorageFixture: public Test
{
public:
  SqliteStorageFixture(): storage_("test_database") {
    std::remove("test_database");
  }

  SqliteStorage storage_;
};

TEST_F(SqliteStorageFixture, write_single_message_to_storage) {
  storage_.open(true);
  storage_.write("test_message");
  storage_.close();

  sqlite::DBPtr db = sqlite::open("test_database");
  auto messages = sqlite::getMessages(db);
  sqlite::close(db);

  ASSERT_THAT(messages, SizeIs(1));
  ASSERT_THAT(messages[0], Eq("test_message"));
}

TEST_F(SqliteStorageFixture, open_fails_if_database_already_exists) {
  storage_.open();
  storage_.write("test_message");
  storage_.close();

  bool result = storage_.open();

  EXPECT_THAT(result, Eq(false));
}


TEST_F(SqliteStorageFixture, write_fails_if_database_not_opened) {
  bool result = storage_.write("test_message");

  EXPECT_THAT(result, Eq(false));
}
