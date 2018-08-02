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

#include <cstring>
#include <iostream>
#include <string>
#include <memory>
#include <vector>

namespace rosbag2_storage_plugins
{

SqliteWrapper::SqliteWrapper(const std::string & filename)
: db_ptr(nullptr)
{
  int rc = sqlite3_open(filename.c_str(), &db_ptr);
  if (rc) {
    throw SqliteException("Could not open database. Error: " + std::string(sqlite3_errmsg(db_ptr)));
  }
}

SqliteWrapper::SqliteWrapper()
: db_ptr(nullptr) {}

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

bool SqliteWrapper::get_message(void * buffer, size_t & size, size_t index)
{
  std::string selected_message;
  std::string offset = std::to_string(index);
  sqlite3_stmt * statement;
  std::string query = "SELECT * FROM messages LIMIT 1 OFFSET " + offset;

  int return_code = sqlite3_prepare_v2(db_ptr, query.c_str(), -1, &statement, nullptr);
  if (return_code != SQLITE_OK) {
    throw SqliteException("SQL error when preparing statement '" + query + "'with return code: " +
            std::to_string(return_code));
  }

  int result = sqlite3_step(statement);
  if (result == SQLITE_ROW) {
    selected_message =
      std::string(reinterpret_cast<const char *>(sqlite3_column_text(statement, 1)));
    size = strlen(selected_message.c_str());
    memcpy(buffer, selected_message.c_str(), size);
    sqlite3_finalize(statement);
    return true;
  } else {
    sqlite3_finalize(statement);
    return false;
  }
}

SqliteWrapper::operator bool()
{
  return db_ptr != nullptr;
}

}  // namespace rosbag2_storage_plugins
