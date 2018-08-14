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
#include <vector>
#include <fstream>

#include "../../../src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"
#include "mock_sqlite_wrapper.hpp"
#include "mock_sqlite_statement_wrapper.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_storage_plugins;  // NOLINT

TEST(SqliteStorageTest, write_single_message_to_storage) {
  auto sqlite_wrapper = std::make_shared<NiceMock<MockSqliteWrapper>>();
  auto statement_wrapper = std::make_shared<MockSqliteStatementWrapper>();
  EXPECT_CALL(*sqlite_wrapper, get_prepared_statement(_)).WillRepeatedly(Return(statement_wrapper));

  EXPECT_CALL(*statement_wrapper, bind_text(1, _, _));
  EXPECT_CALL(*statement_wrapper, bind_int(2, _));
  EXPECT_CALL(*statement_wrapper, step());

  rosbag2_storage::SerializedBagMessage test_message;
  test_message.serialized_data = std::make_shared<rcutils_char_array_t>();
  auto storage = std::make_unique<SqliteStorage>(sqlite_wrapper);
  storage->write(test_message);
}

TEST(SqliteStorageTest, read_messages_from_storage_reads_correct_columns) {
  auto sqlite_wrapper = std::make_shared<NiceMock<MockSqliteWrapper>>();
  auto statement_wrapper = std::make_shared<NiceMock<MockSqliteStatementWrapper>>();
  EXPECT_CALL(*sqlite_wrapper, get_prepared_statement(_)).WillRepeatedly(Return(statement_wrapper));

  rosbag2_storage::SerializedBagMessage test_message;
  test_message.serialized_data = std::make_shared<rcutils_char_array_t>();
  auto storage = std::make_unique<SqliteStorage>(sqlite_wrapper);
  storage->write(test_message);

  int text_column = 0;
  int time_stamp_column = 1;

  EXPECT_CALL(*statement_wrapper, read_text_size(text_column));
  EXPECT_CALL(*statement_wrapper, read_text(text_column, _, _));
  EXPECT_CALL(*statement_wrapper, read_int(time_stamp_column));

  storage->read_next();
}

TEST(SqliteStorageTest, has_next_return_false_if_there_is_no_other_message) {
  auto sqlite_wrapper = std::make_shared<NiceMock<MockSqliteWrapper>>();
  auto statement_wrapper = std::make_shared<NiceMock<MockSqliteStatementWrapper>>();
  EXPECT_CALL(*sqlite_wrapper, get_prepared_statement(_)).WillRepeatedly(Return(statement_wrapper));

  auto storage = std::make_unique<SqliteStorage>(sqlite_wrapper);
  int rows = 2;
  std::string query = "SELECT COALESCE(MAX(id), 0) FROM messages";
  EXPECT_CALL(*sqlite_wrapper, execute_query(query, _, _))
  .WillRepeatedly(SetSecondArgumentVoidPointerToInt(&rows));

  EXPECT_TRUE(storage->has_next());
  storage->read_next();
  EXPECT_TRUE(storage->has_next());
  storage->read_next();
  EXPECT_FALSE(storage->has_next());
}
