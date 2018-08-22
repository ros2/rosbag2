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

std::shared_ptr<SqliteStatementWrapper> SqliteWrapper::prepare_statement(std::string query)
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
