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

#include "rcutils/types.h"

#include "../../../src/rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.hpp"

#include "storage_test_fixture.hpp"

using namespace ::testing;  // NOLINT

TEST_F(StorageTestFixture, querying_empty_adhoc_results_gives_empty_query_result) {
  auto db = rosbag2_storage_plugins::SqliteWrapper{database_name_};

  auto result = db.prepare_statement("SELECT 1 WHERE 1=2;")->execute_query<int>();

  ASSERT_THAT(result.begin(), Eq(result.end()));
}

TEST_F(StorageTestFixture, querying_adhoc_results_with_normal_data_gives_content) {
  auto db = rosbag2_storage_plugins::SqliteWrapper{database_name_};

  auto result =
    db.prepare_statement("SELECT 1, 1.465, 'abc';")->execute_query<int, double, std::string>();
  auto row = *(result.begin());

  ASSERT_THAT(std::get<0>(row), Eq(1));
  ASSERT_THAT(std::get<1>(row), Eq(1.465));
  ASSERT_THAT(std::get<2>(row), StrEq("abc"));
}

TEST_F(StorageTestFixture, bind_values_are_inserted) {
  auto db = rosbag2_storage_plugins::SqliteWrapper{database_name_};
  db.prepare_statement("CREATE TABLE test (int_col INTEGER, double_col FLOAT, string_col TEXT);")
  ->execute_and_reset();

  auto statement = db.prepare_statement(
    "INSERT INTO test (int_col, double_col, string_col) VALUES (?, ?, ?);");
  statement->bind(1, 3.14, "abc");
  statement->execute_and_reset();

  auto row_iter = db.prepare_statement(
    "SELECT COUNT(*) FROM test WHERE int_col = 1 AND double_col = 3.14 AND string_col = 'abc';")
    ->execute_query<int>().begin();
  ASSERT_THAT(std::get<0>(*row_iter), Eq(1));
}

TEST_F(StorageTestFixture, reuse_prepared_statement) {
  auto db = rosbag2_storage_plugins::SqliteWrapper{database_name_};
  db.prepare_statement("CREATE TABLE test (col INTEGER);")->execute_and_reset();

  auto statement = db.prepare_statement("INSERT INTO test (col) VALUES (?);");
  statement->bind(1);
  statement->execute_and_reset();
  statement->bind(2);
  statement->execute_and_reset();

  auto row_iter = db.prepare_statement("SELECT COUNT(*) FROM test;")->execute_query<int>().begin();
  ASSERT_THAT(std::get<0>(*row_iter), Eq(2));
}

TEST_F(StorageTestFixture, all_result_rows_are_available) {
  auto db = rosbag2_storage_plugins::SqliteWrapper{database_name_};
  db.prepare_statement("CREATE TABLE test (col INTEGER);")->execute_and_reset();
  db.prepare_statement("INSERT INTO test (col) VALUES (1);")->execute_and_reset();
  db.prepare_statement("INSERT INTO test (col) VALUES (2);")->execute_and_reset();
  db.prepare_statement("INSERT INTO test (col) VALUES (3);")->execute_and_reset();

  auto result =
    db.prepare_statement("SELECT col FROM test ORDER BY col ASC;")->execute_query<int>();

  // range-for access
  int row_value = 1;
  for (auto row : result) {
    ASSERT_THAT(std::get<0>(row), Eq(row_value++));
  }
  ASSERT_THAT(row_value, Eq(4));

  // iterator access
  row_value = 1;
  auto row_iterator = result.begin();
  while (row_iterator != result.end()) {
    ASSERT_THAT(std::get<0>(*row_iterator), Eq(row_value++));
    ++row_iterator;
  }
  ASSERT_THAT(row_value, Eq(4));
}

TEST_F(StorageTestFixture, ros_specific_types_are_supported_for_reading_and_writing) {
  auto db = rosbag2_storage_plugins::SqliteWrapper{database_name_};
  db.prepare_statement("CREATE TABLE test (timestamp INTEGER, data BLOB);")->execute_and_reset();
  rcutils_time_point_value_t time = 1099511627783;
  auto msg_content = "message";
  std::shared_ptr<rcutils_char_array_t> message = make_serialized_message(msg_content);

  auto statement = db.prepare_statement("INSERT INTO test (timestamp, data) VALUES (?, ?);");
  statement->bind(time, message);
  statement->execute_and_reset();
  auto row_iter = db.prepare_statement("SELECT timestamp, data FROM test")
    ->execute_query<rcutils_time_point_value_t, std::shared_ptr<rcutils_char_array_t>>().begin();

  ASSERT_THAT(std::get<0>(*row_iter), Eq(time));
  ASSERT_THAT(deserialize_message(std::get<1>(*row_iter)), StrEq(msg_content));
}
