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

#include "rcpputils/filesystem_helper.hpp"

#include "rcutils/types.h"

#include "rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.hpp"

#include "storage_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class SqliteWrapperTestFixture : public StorageTestFixture
{
public:
  SqliteWrapperTestFixture()
  : StorageTestFixture(),
    db_(
      (rcpputils::fs::path(temporary_dir_path_) / "test.db3").string(),
      rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE)
  {}

  rosbag2_storage_plugins::SqliteWrapper db_;
};

TEST_F(SqliteWrapperTestFixture, querying_empty_adhoc_results_gives_empty_query_result) {
  auto result = db_.prepare_statement("SELECT 1 WHERE 1=2;")->execute_query<int>();

  // ASSERT_THAT cannot be used as the iterators can only be moved and not copied.
  ASSERT_TRUE(result.begin() == result.end());
}

TEST_F(SqliteWrapperTestFixture, querying_adhoc_results_with_normal_data_gives_content) {
  auto result =
    db_.prepare_statement("SELECT 1, 1.465, 'abc';")->execute_query<int, double, std::string>();
  auto row_iter = result.begin();

  ASSERT_THAT(std::get<0>(*row_iter), Eq(1));
  ASSERT_THAT(std::get<1>(*row_iter), Eq(1.465));
  ASSERT_THAT(std::get<2>(*row_iter), StrEq("abc"));
}

TEST_F(SqliteWrapperTestFixture, bind_values_are_inserted) {
  db_.prepare_statement("CREATE TABLE test (int_col INTEGER, double_col FLOAT, string_col TEXT);")
  ->execute_and_reset();

  db_.prepare_statement("INSERT INTO test (int_col, double_col, string_col) VALUES (?, ?, ?);")
  ->bind(1, 3.14, "abc")->execute_and_reset();

  auto row_iter = db_.prepare_statement(
    "SELECT COUNT(*) FROM test WHERE int_col = 1 AND double_col = 3.14 AND string_col = 'abc';")
    ->execute_query<int>().begin();
  ASSERT_THAT(std::get<0>(*row_iter), Eq(1));
}

TEST_F(SqliteWrapperTestFixture, reuse_prepared_statement) {
  db_.prepare_statement("CREATE TABLE test (col INTEGER);")->execute_and_reset();

  db_.prepare_statement("INSERT INTO test (col) VALUES (?);")
  ->bind(1)->execute_and_reset()->bind(2)->execute_and_reset();

  auto row_iter = db_.prepare_statement("SELECT COUNT(*) FROM test;")->execute_query<int>().begin();
  ASSERT_THAT(std::get<0>(*row_iter), Eq(2));
}

TEST_F(SqliteWrapperTestFixture, all_result_rows_are_available) {
  db_.prepare_statement("CREATE TABLE test (col INTEGER);")->execute_and_reset();
  db_.prepare_statement("INSERT INTO test (col) VALUES (1);")->execute_and_reset();
  db_.prepare_statement("INSERT INTO test (col) VALUES (2);")->execute_and_reset();
  db_.prepare_statement("INSERT INTO test (col) VALUES (3);")->execute_and_reset();

  auto query = db_.prepare_statement("SELECT col FROM test ORDER BY col ASC;");
  auto result = query->execute_query<int>();

  // range-for access
  int row_value = 1;
  for (auto row : result) {
    ASSERT_THAT(std::get<0>(row), Eq(row_value++));
  }
  ASSERT_THAT(row_value, Eq(4));

  query->reset();
  result = query->execute_query<int>();

  // iterator access
  row_value = 1;
  auto row_iterator = result.begin();
  while (row_iterator != result.end()) {
    ASSERT_THAT(std::get<0>(*row_iterator), Eq(row_value++));
    ++row_iterator;
  }
  ASSERT_THAT(row_value, Eq(4));
}

TEST_F(SqliteWrapperTestFixture, only_a_single_iterator_is_allowed_per_result) {
  auto result = db_.prepare_statement("SELECT 1;")->execute_query<int>();

  auto row = result.begin();

  EXPECT_THROW(result.get_single_line(), rosbag2_storage_plugins::SqliteException);
  EXPECT_THROW(result.begin(), rosbag2_storage_plugins::SqliteException);
  EXPECT_NO_THROW(result.end());
}

TEST_F(SqliteWrapperTestFixture, ros_specific_types_are_supported_for_reading_and_writing) {
  db_.prepare_statement("CREATE TABLE test (timestamp INTEGER, data BLOB);")->execute_and_reset();
  rcutils_time_point_value_t time = 1099511627783;
  auto msg_content = "message";
  std::shared_ptr<rcutils_uint8_array_t> message = make_serialized_message(msg_content);

  db_.prepare_statement("INSERT INTO test (timestamp, data) VALUES (?, ?);")
  ->bind(time, message)->execute_and_reset();
  auto row_iter = db_.prepare_statement("SELECT timestamp, data FROM test")
    ->execute_query<rcutils_time_point_value_t, std::shared_ptr<rcutils_uint8_array_t>>().begin();

  ASSERT_THAT(std::get<0>(*row_iter), Eq(time));
  ASSERT_THAT(deserialize_message(std::get<1>(*row_iter)), StrEq(msg_content));
}

TEST_F(SqliteWrapperTestFixture, single_line_results_can_be_obtained_directly) {
  auto row = db_.prepare_statement("SELECT 1, 2;")->execute_query<int, int>().get_single_line();

  ASSERT_THAT(std::get<0>(row), Eq(1));
  ASSERT_THAT(std::get<1>(row), Eq(2));
}

TEST_F(SqliteWrapperTestFixture, get_single_line_handles_empty_result_set) {
  auto result = db_.prepare_statement("SELECT 1, 2 WHERE 1 = 2;")->execute_query<int, int>();

  EXPECT_THROW(result.get_single_line(), rosbag2_storage_plugins::SqliteException);
}

TEST_F(SqliteWrapperTestFixture, field_exists) {
  db_.prepare_statement("CREATE TABLE test_table (timestamp INTEGER, data BLOB);")
  ->execute_and_reset();
  rcutils_time_point_value_t time = 1099511627783;
  auto msg_content = "message";
  std::shared_ptr<rcutils_uint8_array_t> message = make_serialized_message(msg_content);

  db_.prepare_statement("INSERT INTO test_table (timestamp, data) VALUES (?, ?);")
  ->bind(time, message)->execute_and_reset();

  EXPECT_TRUE(db_.field_exists("test_table", "data"));
  EXPECT_FALSE(db_.field_exists("test_table", "non_existent_field"));
  EXPECT_THROW(
    db_.field_exists("non_existent_table", "data"), rosbag2_storage_plugins::SqliteException);
}

TEST_F(SqliteWrapperTestFixture, table_exists) {
  db_.prepare_statement("CREATE TABLE test_table (timestamp INTEGER, data BLOB);")
  ->execute_and_reset();
  EXPECT_TRUE(db_.table_exists("test_table"));
  EXPECT_FALSE(db_.table_exists("non_existent_table"));
}
