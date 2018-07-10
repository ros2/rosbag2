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

#include "sqlite_wrapper.hpp"

#include <string>
#include <memory>
#include <vector>

#include "sqlite_statement.hpp"

namespace rosbag2
{

SqliteWrapper::SqliteWrapper(const std::string & filename)
: db_ptr(nullptr)
{
  int rc = sqlite3_open(filename.c_str(), &db_ptr);
  if (rc) {
    throw SqliteException("Could not open database");
  }
}

SqliteWrapper::~SqliteWrapper()
{
  sqlite3_close(db_ptr);
}

void SqliteWrapper::execute_query(const std::string & query)
{
  char * error_msg = nullptr;
  int return_code = sqlite3_exec(db_ptr, query.c_str(), nullptr, nullptr, &error_msg);

  if (return_code != SQLITE_OK) {
    auto error = "SQL error: " + std::string(error_msg);
    sqlite3_free(error_msg);
    throw SqliteException(error);
  }
}

SqliteStatement SqliteWrapper::prepare_statement(const std::string & query)
{
  return SqliteStatement(db_ptr, query);
}

std::vector<std::string> SqliteWrapper::get_raw_messages(const std::string & query)
{
  std::vector<std::string> table_msgs;
  SqliteStatement statement = prepare_statement(query);
  int result = sqlite3_step(statement.get());
  while (result == SQLITE_ROW) {
    table_msgs.emplace_back(
      reinterpret_cast<const char *>(sqlite3_column_text(statement.get(), 1)));
    result = sqlite3_step(statement.get());
  }

  return table_msgs;
}

SqliteWrapper::operator bool()
{
  return static_cast<bool>(db_ptr);
}

}  // namespace rosbag2
