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

#include "sqlite_exception.hpp"

namespace rosbag2_storage_plugins
{

SqliteStatementWrapper::SqliteStatementWrapper(sqlite3 * database, std::string query)
{
  sqlite3_stmt * statement;
  int return_code = sqlite3_prepare_v2(database, query.c_str(), -1, &statement, nullptr);
  if (return_code != SQLITE_OK) {
    throw SqliteException("SQL error when preparing statement '" + query + "' with return code: " +
            std::to_string(return_code));
  }

  statement_ = statement;
  last_bound_parameter_index_ = 0;
}

SqliteStatementWrapper::~SqliteStatementWrapper()
{
  if (statement_) {
    sqlite3_finalize(statement_);
  }
}

void SqliteStatementWrapper::execute_and_reset()
{
  int return_code = sqlite3_step(statement_);
  if (return_code != SQLITE_OK && return_code != SQLITE_DONE) {
    throw SqliteException("Error processing SQLite statement.");
  }
  reset();
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

void SqliteStatementWrapper::reset()
{
  sqlite3_reset(statement_);
  sqlite3_clear_bindings(statement_);
  last_bound_parameter_index_ = 0;
  written_blobs_cache_.clear();
}

bool SqliteStatementWrapper::step()
{
  int return_code = sqlite3_step(statement_);
  if (return_code == SQLITE_ROW) {
    return true;
  } else if (return_code == SQLITE_DONE) {
    return false;
  } else {
    throw SqliteException("Error reading query result: " + std::to_string(return_code));
  }
}

void SqliteStatementWrapper::obtain_column_value(size_t index, int & value) const
{
  value = sqlite3_column_int(statement_, static_cast<int>(index));
}

void
SqliteStatementWrapper::obtain_column_value(size_t index, rcutils_time_point_value_t & value) const
{
  value = sqlite3_column_int64(statement_, static_cast<int>(index));
}

void SqliteStatementWrapper::obtain_column_value(size_t index, double & value) const
{
  value = sqlite3_column_double(statement_, static_cast<int>(index));
}

void SqliteStatementWrapper::obtain_column_value(size_t index, std::string & value) const
{
  value = reinterpret_cast<const char *>(sqlite3_column_text(statement_, static_cast<int>(index)));
}

void SqliteStatementWrapper::obtain_column_value(
  size_t index, std::shared_ptr<rcutils_char_array_t> & value) const
{
  auto data = sqlite3_column_blob(statement_, static_cast<int>(index));
  auto size = static_cast<size_t>(sqlite3_column_bytes(statement_, static_cast<int>(index)));

  auto rcutils_allocator = new rcutils_allocator_t;
  *rcutils_allocator = rcutils_get_default_allocator();
  auto msg = new rcutils_char_array_t;
  *msg = rcutils_get_zero_initialized_char_array();
  auto ret = rcutils_char_array_init(msg, size, rcutils_allocator);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("Error allocating resources for serialized message: " +
            std::string(rcutils_get_error_string_safe()));
  }

  value = std::shared_ptr<rcutils_char_array_t>(msg,
      [](rcutils_char_array_t * msg) {
        int error = rcutils_char_array_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2_storage_default_plugins",
            "Leaking memory. Error: %s", rcutils_get_error_string_safe());
        }
      });

  memcpy(value->buffer, data, size);
  value->buffer_length = size;
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
