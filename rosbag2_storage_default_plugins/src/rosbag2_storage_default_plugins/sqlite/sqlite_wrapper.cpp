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

#include "rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rcutils/types.h"
#include "rosbag2_storage/serialized_bag_message.hpp"

#include "rosbag2_storage_default_plugins/sqlite/sqlite_exception.hpp"

#include "../logging.hpp"

namespace rosbag2_storage_plugins
{

SqliteWrapper::SqliteWrapper(
  const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag io_flag)
: db_ptr(nullptr)
{
  if (io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    int rc = sqlite3_open_v2(uri.c_str(), &db_ptr,
        SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, nullptr);
    if (rc != SQLITE_OK) {
      throw SqliteException("Could not read-only open database. Error code: " + std::to_string(rc));
    }
  } else {
    int rc = sqlite3_open_v2(uri.c_str(), &db_ptr,
        SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX, nullptr);
    if (rc != SQLITE_OK) {
      throw SqliteException(
              "Could not read-write open database. Error code: " + std::to_string(rc));
    }
    prepare_statement("PRAGMA journal_mode = WAL;")->execute_and_reset();
    prepare_statement("PRAGMA synchronous = NORMAL;")->execute_and_reset();
  }
}

SqliteWrapper::SqliteWrapper()
: db_ptr(nullptr) {}

SqliteWrapper::~SqliteWrapper()
{
  int rc = sqlite3_close(db_ptr);
  if (rc != SQLITE_OK) {
    ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_ERROR_STREAM(
      "Could not close open database. Error code: " + std::to_string(rc) +
      " Error message: " + sqlite3_errstr(rc));
  }
}

SqliteStatement SqliteWrapper::prepare_statement(const std::string & query)
{
  return std::make_shared<SqliteStatementWrapper>(db_ptr, query);
}

size_t SqliteWrapper::get_last_insert_id()
{
  return sqlite3_last_insert_rowid(db_ptr);
}

SqliteWrapper::operator bool()
{
  return db_ptr != nullptr;
}

}  // namespace rosbag2_storage_plugins
