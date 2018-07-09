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
#include <string>
#include <vector>
#include <fstream>

#include "../../../src/rosbag2/storage/sqlite/sqlite_storage.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT

class SqliteStorageFixture : public Test
{
public:
  SqliteStorageFixture()
  : database_name_(UnitTest::GetInstance()->current_test_info()->name())
  {
    std::remove(database_name_.c_str());
  }

  std::string database_name_;
};

std::vector<std::string> getMessagesFromSqliteDatabase(const std::string & database_name)
{
  sqlite::DBPtr db = sqlite::open(database_name);
  auto messages = sqlite::getMessages(db);
  sqlite::close(db);
  return messages;
}


TEST_F(SqliteStorageFixture, write_single_message_to_storage) {
  auto storage = std::make_unique<SqliteStorage>(database_name_, true);
  storage->write("test_message");
  storage.reset();

  auto messages = getMessagesFromSqliteDatabase(database_name_);

  ASSERT_THAT(messages, SizeIs(1));
  ASSERT_THAT(messages[0], Eq("test_message"));
}

TEST_F(SqliteStorageFixture, create_fails_if_database_already_exists) {
  std::ofstream file {database_name_};

  auto storage = std::make_unique<SqliteStorage>(database_name_, true);

  EXPECT_THAT(storage->is_open(), Eq(false));
}


TEST_F(SqliteStorageFixture, write_fails_if_database_not_opened) {
  std::ofstream file {database_name_};
  auto storage = std::make_unique<SqliteStorage>(database_name_, true);
  EXPECT_THAT(storage->is_open(), Eq(false));

  bool result = storage->write("test_message");

  EXPECT_THAT(result, Eq(false));
}
