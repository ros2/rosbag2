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
#include <string>
#include <utility>

namespace rosbag2_storage_plugins
{

SqliteStatementWrapper::SqliteStatementWrapper()
: statement_(nullptr) {}

SqliteStatementWrapper::SqliteStatementWrapper(sqlite3 * database, std::string query)
{
  sqlite3_stmt * statement;
  int return_code = sqlite3_prepare_v2(database, query.c_str(), -1, &statement, nullptr);
  if (return_code != SQLITE_OK) {
    throw SqliteException("SQL error when preparing statement '" + query + "'with return code: " +
            std::to_string(return_code));
  }

  statement_ = statement;
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

void SqliteStatementWrapper::step()
{
  int unused = sqlite3_step(statement_);
  (void) unused;
}

void SqliteStatementWrapper::step_next_row()
{
  int row = sqlite3_step(statement_);
  if (row != SQLITE_ROW) {
    throw SqliteException("No more rows available in table.");
  }
}

void SqliteStatementWrapper::bind_text(int column, char * buffer, size_t buffer_length)
{
  int return_code = sqlite3_bind_text(
    statement_, column, buffer, static_cast<int>(buffer_length), SQLITE_STATIC);

  if (return_code != SQLITE_OK) {
    throw SqliteException("SQL error when binding text buffer. Return code: " +
            std::to_string(return_code));
  }
}

void SqliteStatementWrapper::bind_int(int column, int64_t int_to_bind)
{
  int return_code = sqlite3_bind_int64(statement_, column, int_to_bind);

  if (return_code != SQLITE_OK) {
    throw SqliteException("SQL error when binding integer. Return code: " +
            std::to_string(return_code));
  }
}

void SqliteStatementWrapper::read_text(int column, char * buffer, size_t size)
{
  memcpy(reinterpret_cast<unsigned char *>(buffer), sqlite3_column_text(statement_, column), size);
}

int64_t SqliteStatementWrapper::read_int(int column)
{
  return sqlite3_column_int64(statement_, column);
}

size_t SqliteStatementWrapper::read_text_size(int column)
{
  return sqlite3_column_bytes(statement_, column);
}

}  // namespace rosbag2_storage_plugins
