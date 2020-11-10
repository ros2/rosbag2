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
#include <vector>

#include "rcutils/types.h"
#include "rosbag2_storage/serialized_bag_message.hpp"

#include "rosbag2_storage_default_plugins/sqlite/sqlite_exception.hpp"

#include "../logging.hpp"

namespace rosbag2_storage_plugins
{

SqliteWrapper::SqliteWrapper(
  const std::string & uri,
  rosbag2_storage::storage_interfaces::IOFlag io_flag,
  std::vector<std::string> && pragmas)
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
  std::vector<std::string> & pragmas,
  rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  // sqlite pragmas are assigned with either '= value' or '(value)' syntax, depending on pragma
  const std::string pragma_assign = "=";
  const std::string pragma_bracket = "(";

  // Apply default pragmas if not overridden by user setting
  {
    // Used to check whether db is readable
    const std::string schema = "schema_version";

    typedef std::unordered_map<std::string, std::string> pragmas_map_t;
    pragmas_map_t default_pragmas = {
      {schema, ""}
    };
    if (io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) {
      const pragmas_map_t write_default_pragmas = {
        {"journal_mode", pragma_assign + "WAL"},
        {"synchronous", pragma_assign + "NORMAL"}
      };
      default_pragmas.insert(write_default_pragmas.begin(), write_default_pragmas.end());
    }
    for (const auto & kv : default_pragmas) {
      auto key = kv.first;
      auto default_assignment = kv.second;
      auto pragma_in_settings = std::find_if(
        pragmas.begin(), pragmas.end(),
        [key](const auto & pragma) {
          return pragma.rfind(key, 0) == 0;
        });
      if (pragma_in_settings == pragmas.end()) {
        pragmas.push_back(key + default_assignment);
      }
    }
  }

  for (auto & pragma : pragmas) {
    if (pragma.empty()) {
      continue;
    }

    // Extract pragma name
    auto pragma_name = pragma;
    auto found_value_assignment = pragma.find(pragma_assign);
    if (found_value_assignment == std::string::npos) {
      found_value_assignment = pragma.find(pragma_bracket);
    }
    if (found_value_assignment != std::string::npos) {
      if (found_value_assignment == 0) {
        // Incorrect syntax, starts with = or (
        std::stringstream errmsg;
        errmsg << "Incorrect storage setting syntax: " << pragma;
        throw SqliteException{errmsg.str()};
      }
      // Strip value assignment part, trim trailing whitespaces before = or (
      pragma_name = pragma.substr(0, found_value_assignment);
      const std::string trailing_set(" \t\f\v\n\r");
      pragma_name = pragma_name.substr(0, pragma_name.find_last_not_of(trailing_set) + 1);
    }

    // Apply the setting. Note that statements that assign value do not reliably return value
    const std::string pragma_keyword = "PRAGMA";
    const std::string pragma_statement = pragma_keyword + " " + pragma + ";";
    prepare_statement(pragma_statement)->execute_and_reset();

    // Check if the value is set, reading the pragma
    auto statement_for_check = pragma_keyword + " " + pragma_name + ";";
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
