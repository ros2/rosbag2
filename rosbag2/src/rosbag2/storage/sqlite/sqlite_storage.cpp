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

#include "sqlite_storage.hpp"

namespace rosbag2
{

SqliteStorage::SqliteStorage(std::string database_name)
: database_(), database_name_(database_name)
{}

SqliteStorage::~SqliteStorage()
{
  close();
}

void SqliteStorage::open()
{
  database_ = sqlite::open(database_name_);

  std::string create_table = "CREATE TABLE IF NOT EXISTS messages(" \
    "id INTEGER PRIMARY KEY AUTOINCREMENT," \
    "data           BLOB    NOT NULL," \
    "timestamp      INT     NOT NULL);";

  sqlite::execute_query(database_, create_table);
}

void SqliteStorage::insertMessage(std::string data)
{
  std::string insert_message =
    "INSERT INTO messages (data, timestamp) VALUES ('" + data + "', strftime('%s%f','now'))";
  sqlite::execute_query(database_, insert_message);
}

void SqliteStorage::close()
{
  if (database_) {
    sqlite::close(database_);
  }
}

}  // namespace rosbag2
