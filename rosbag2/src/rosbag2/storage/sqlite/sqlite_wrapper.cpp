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

#include <string>
#include <memory>
#include <vector>

#include "sqlite_wrapper.hpp"

namespace rosbag2
{

SqliteWrapper::SqliteWrapper(const std::string & filename)
{
  int rc = sqlite3_open(filename.c_str(), &db_ptr);
  if (rc) {
    db_ptr = nullptr;
  }
}

SqliteWrapper::~SqliteWrapper()
{
  if (db_ptr) {
    sqlite3_close(db_ptr);
  }
}

void SqliteWrapper::execute_query(const std::string & query)
{
  char * error_msg = 0;
  int return_code = sqlite3_exec(db_ptr, query.c_str(), nullptr, nullptr, &error_msg);

  if (return_code != SQLITE_OK) {
    auto error = "SQL error: " + std::string(error_msg);
    sqlite3_free(error_msg);
    throw sql_exception(error);
  }
}

std::vector<std::string> SqliteWrapper::get_messages(std::string table)
{
  std::vector<std::string> table_msgs;
  sqlite3_stmt * statement;
  std::string query = "SELECT * FROM " + table;
  sqlite3_prepare_v2(db_ptr, query.c_str(), -1, &statement, nullptr);
  int result = sqlite3_step(statement);
  while (result == SQLITE_ROW) {
    table_msgs.emplace_back(
      std::string(reinterpret_cast<const char *>(sqlite3_column_text(statement, 1))));
    result = sqlite3_step(statement);
  }
  sqlite3_finalize(statement);

  return table_msgs;
}

SqliteWrapper::operator bool()
{
  return static_cast<bool>(db_ptr);
}

}  // namespace rosbag2
