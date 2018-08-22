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

#include "sqlite_statement_wrapper.hpp"

#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/logging_macros.h"

namespace rosbag2_storage_plugins
{

SqliteStatementWrapper::SqliteStatementWrapper()
: statement_(nullptr), is_prepared_(false), last_bound_parameter_index_(0) {}

SqliteStatementWrapper::SqliteStatementWrapper(sqlite3 * database, std::string query)
{
  sqlite3_stmt * statement;
  int return_code = sqlite3_prepare_v2(database, query.c_str(), -1, &statement, nullptr);
  if (return_code != SQLITE_OK) {
    throw SqliteException("SQL error when preparing statement '" + query + "'with return code: " +
            std::to_string(return_code));
  }

  statement_ = statement;
  is_prepared_ = true;
  last_bound_parameter_index_ = 0;
}

SqliteStatementWrapper::~SqliteStatementWrapper()
{
  if (statement_) {
    sqlite3_finalize(statement_);
  }
}

sqlite3_stmt * SqliteStatementWrapper::get()
{
  return statement_;
}

void SqliteStatementWrapper::execute_and_reset()
{
  int return_code = sqlite3_step(statement_);
  if (return_code != SQLITE_OK && return_code != SQLITE_DONE) {
    throw SqliteException("Error processing SQLite statement.");
  }
  reset();
}

void SqliteStatementWrapper::advance_one_row()
{
  int row = sqlite3_step(statement_);
  if (row != SQLITE_ROW) {
    throw SqliteException("No more rows available in table.");
  }
}

void SqliteStatementWrapper::bind(int value)
{
  auto return_code = sqlite3_bind_int(statement_, ++last_bound_parameter_index_, value);
  check_and_report_bind_error(return_code, value);
}

void SqliteStatementWrapper::bind(rcutils_time_point_value_t value)
{
  auto return_code = sqlite3_bind_int64(statement_, ++last_bound_parameter_index_, value);
  check_and_report_bind_error(return_code, value);
}

void SqliteStatementWrapper::bind(double value)
{
  auto return_code = sqlite3_bind_double(statement_, ++last_bound_parameter_index_, value);
  check_and_report_bind_error(return_code, value);
}

void SqliteStatementWrapper::bind(std::string value)
{
  auto return_code = sqlite3_bind_text(
    statement_, ++last_bound_parameter_index_, value.c_str(), -1, SQLITE_TRANSIENT);
  check_and_report_bind_error(return_code, value);
}

void SqliteStatementWrapper::bind(std::shared_ptr<rcutils_char_array_t> value)
{
  written_blobs_cache_.push_back(value);
  auto return_code = sqlite3_bind_blob(
    statement_, ++last_bound_parameter_index_,
    value->buffer, static_cast<int>(value->buffer_length), SQLITE_STATIC);
  check_and_report_bind_error(return_code);
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> SqliteStatementWrapper::read_table_entry(
  int blob_column, int timestamp_column)
{
  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  int buffer_size = sqlite3_column_bytes(statement_, blob_column);

  auto rcutils_allocator = new rcutils_allocator_t;
  *rcutils_allocator = rcutils_get_default_allocator();
  auto msg = new rcutils_char_array_t;
  *msg = rcutils_get_zero_initialized_char_array();
  auto ret = rcutils_char_array_init(msg, buffer_size, rcutils_allocator);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("Error allocating resources for serialized message" +
            std::string(rcutils_get_error_string_safe()));
  }

  message->serialized_data = std::shared_ptr<rcutils_char_array_t>(msg,
      [](rcutils_char_array_t * msg) {
        int error = rcutils_char_array_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2_storage_default_plugins",
            "Leaking memory. Error: %s", rcutils_get_error_string_safe());
        }
      });

  memcpy(
    reinterpret_cast<unsigned char *>(message->serialized_data->buffer),
    sqlite3_column_blob(statement_, blob_column),
    buffer_size);
  message->serialized_data->buffer_length = buffer_size;
  message->time_stamp = sqlite3_column_int64(statement_, timestamp_column);

  return message;
}

void SqliteStatementWrapper::reset()
{
  sqlite3_reset(statement_);
  sqlite3_clear_bindings(statement_);
  last_bound_parameter_index_ = 0;
  written_blobs_cache_.clear();
}

bool SqliteStatementWrapper::is_prepared()
{
  return is_prepared_;
}

void SqliteStatementWrapper::check_and_report_bind_error(int return_code)
{
  if (return_code != SQLITE_OK) {
    throw SqliteException("SQLite error when binding parameter " +
            std::to_string(last_bound_parameter_index_) + ". Return code: " +
            std::to_string(return_code));
  }
}

}  // namespace rosbag2_storage_plugins
