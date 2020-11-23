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

#include <algorithm>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

#include "rcutils/types.h"
#include "rosbag2_storage/serialized_bag_message.hpp"

#include "rosbag2_storage_default_plugins/sqlite/sqlite_exception.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_pragmas.hpp"

#include "../logging.hpp"

namespace rosbag2_storage_plugins
{

SqliteWrapper::SqliteWrapper(
  const std::string & uri,
  rosbag2_storage::storage_interfaces::IOFlag io_flag,
  std::unordered_map<std::string, std::string> && pragmas)
: db_ptr(nullptr)
{
  if (io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    int rc = sqlite3_open_v2(
      uri.c_str(), &db_ptr,
      SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, nullptr);
    if (rc != SQLITE_OK) {
      std::stringstream errmsg;
      errmsg << "Could not read-only open database. SQLite error (" <<
        rc << "): " << sqlite3_errstr(rc);
      throw SqliteException{errmsg.str()};
    }
  } else {
    int rc = sqlite3_open_v2(
      uri.c_str(), &db_ptr,
      SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX, nullptr);
    if (rc != SQLITE_OK) {
      std::stringstream errmsg;
      errmsg << "Could not read-write open database. SQLite error (" <<
        rc << "): " << sqlite3_errstr(rc);
      throw SqliteException{errmsg.str()};
    }
  }

  apply_pragma_settings(pragmas, io_flag);
  sqlite3_extended_result_codes(db_ptr, 1);
}

SqliteWrapper::SqliteWrapper()
: db_ptr(nullptr) {}

SqliteWrapper::~SqliteWrapper()
{
  const int rc = sqlite3_close(db_ptr);
  if (rc != SQLITE_OK) {
    ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_ERROR_STREAM(
      "Could not close open database. Error code: " << rc <<
        " Error message: " << sqlite3_errstr(rc));
  }
}

void SqliteWrapper::apply_pragma_settings(
  std::unordered_map<std::string, std::string> & pragmas,
  rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  // Add default pragmas if not overridden by user setting
  {
    auto default_pragmas = SqlitePragmas::default_pragmas();
    if (io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) {
      const auto write_default_pragmas = SqlitePragmas::optimized_writing_pragmas();
      default_pragmas.insert(write_default_pragmas.begin(), write_default_pragmas.end());
    }
    for (const auto & kv : default_pragmas) {
      auto key = kv.first;
      auto default_full_statement = kv.second;
      // insert default if other value not specified for the same key
      if (pragmas.find(key) == pragmas.end()) {
        pragmas.insert(std::make_pair(key, default_full_statement));
      }
    }
  }

  for (auto & kv : pragmas) {
    // Apply the setting. Note that statements that assign value do not reliably return value
    auto pragma_name = kv.first;
    auto pragma_statement = kv.second;
    prepare_statement(pragma_statement)->execute_and_reset();

    // Check if the value is set, reading the pragma
    auto statement_for_check = "PRAGMA " + pragma_name + ";";
    prepare_statement(statement_for_check)->execute_and_reset(true);
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
