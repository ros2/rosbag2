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

#include "sqlite_wrapper.hpp"

#include <cassert>
#include <cstring>
#include <iostream>
#include <string>
#include <memory>
#include <vector>

#include "rcutils/types.h"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_storage_plugins
{

SqliteWrapper::SqliteWrapper(const std::string & uri)
: db_ptr(nullptr)
{
  int rc = sqlite3_open(uri.c_str(), &db_ptr);
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

void SqliteWrapper::execute_query(
  const std::string & query,
  int (* callback)(void *, int, char **, char **),
  void * first_callback_argument)
{
  char * error_msg = nullptr;
  int return_code = sqlite3_exec(
    db_ptr, query.c_str(), callback, first_callback_argument, &error_msg);

  if (return_code != SQLITE_OK) {
    auto error = "SQL error: " + std::string(error_msg);
    sqlite3_free(error_msg);
    throw SqliteException(error);
  }
}

void SqliteWrapper::write_stamped_char_array(char * buffer, size_t buffer_length)
{
  sqlite3_stmt * statement = nullptr;

  std::string query = "INSERT INTO messages (data, timestamp) VALUES (?, strftime('%s%f','now'));";
  int return_code = sqlite3_prepare_v2(db_ptr, query.c_str(), -1, &statement, nullptr);
  if (return_code != SQLITE_OK) {
    throw SqliteException("SQL error when preparing statement '" + query + "'with return code: " +
            std::to_string(return_code));
  }

  return_code = sqlite3_bind_text(statement, 1, buffer, buffer_length, SQLITE_STATIC);

  if (return_code != SQLITE_OK) {
    throw SqliteException("SQL error when binding buffer. Return code: " +
            std::to_string(return_code));
  }

  auto error = sqlite3_step(statement);
  assert(error != SQLITE_ROW);

  sqlite3_finalize(statement);
}

rosbag2_storage::SerializedBagMessage SqliteWrapper::get_message(size_t index)
{
  rosbag2_storage::SerializedBagMessage message;
  std::string offset = std::to_string(index);
  sqlite3_stmt * statement;
  std::string query = "SELECT data FROM messages WHERE id = " + std::to_string(index + 1);

  int return_code = sqlite3_prepare_v2(db_ptr, query.c_str(), -1, &statement, nullptr);
  if (return_code != SQLITE_OK) {
    throw SqliteException("SQL error when preparing statement '" + query + "'with return code: " +
            std::to_string(return_code));
  }

  int result = sqlite3_step(statement);
  unsigned char * read_blob = nullptr;
  if (result == SQLITE_ROW) {
    auto size = sqlite3_column_bytes(statement, 0);
    read_blob = (unsigned char *)malloc(size);
    memcpy(read_blob, sqlite3_column_text(statement, 0), size);
    sqlite3_finalize(statement);
    message.serialized_data = std::make_shared<rcutils_char_array_t>();
    message.serialized_data->buffer = reinterpret_cast<char *>(read_blob);
    message.serialized_data->buffer_capacity = size;
    message.serialized_data->buffer_length = size;
    message.serialized_data->allocator = rcutils_get_default_allocator();
    return message;
  } else {
    sqlite3_finalize(statement);
    throw SqliteException("No more messages available");
  }
}

SqliteWrapper::operator bool()
{
  return db_ptr != nullptr;
}

}  // namespace rosbag2_storage_plugins
