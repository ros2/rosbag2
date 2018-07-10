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
#include "../rosbag2_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT

class MockSqliteWrapper : public SqliteWrapper
{
public:
  explicit MockSqliteWrapper(std::string file_name)
  : SqliteWrapper(""), file_name_(file_name), is_valid_(true) {}

  void execute_query(const std::string & query) override
  {
    std::ofstream mock_database(file_name_);
    if (mock_database.is_open()) {
      mock_database << query;
      mock_database.close();
    }
  }

  explicit operator bool() override
  {
    return is_valid_;
  }

  std::string file_name_;
  bool is_valid_;
};

std::vector<std::string> getWrittenContent(std::string file_name)
{
  std::string file_line;
  std::vector<std::string> content;
  std::ifstream mock_database(file_name);
  if (mock_database.is_open()) {
    while (getline(mock_database, file_line)) {
      content.push_back(file_line);
    }
    mock_database.close();
  }

  return content;
}

TEST_F(Rosbag2TestFixture, write_single_message_to_storage) {
  auto storage = std::make_unique<SqliteStorage>(
    std::make_unique<MockSqliteWrapper>(database_name_));
  storage->write("test_message");
  storage.reset();

  auto messages = getWrittenContent(database_name_);

  ASSERT_THAT(messages, SizeIs(1));
  ASSERT_THAT(messages[0], HasSubstr("'test_message'"));
}

TEST_F(Rosbag2TestFixture, write_fails_if_database_is_not_open) {
  auto storage = std::make_unique<SqliteStorage>(nullptr);
  bool written = storage->write("test_message");
  storage.reset();

  ASSERT_FALSE(written);
}
