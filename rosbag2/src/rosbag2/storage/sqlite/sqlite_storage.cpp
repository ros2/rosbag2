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

#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace rosbag2
{

SqliteStorage::SqliteStorage(const std::string & database_name, bool shouldInitialize)
: database_()
{
  open(database_name);
  if (shouldInitialize) {
    initialize();
  }
}

SqliteStorage::SqliteStorage(std::unique_ptr<SqliteWrapper> database)
: database_(std::move(database))
{}

bool SqliteStorage::is_open()
{
  return static_cast<bool>(database_.get());
}

bool SqliteStorage::write(const std::string & data)
{
  if (!is_open()) {
    std::cerr << "Failed to write message. The database is not open." << std::endl;
    return false;
  }

  try {
    std::string insert_message =
      "INSERT INTO messages (data, timestamp) VALUES ('" + data + "', strftime('%s%f','now'))";
    database_->execute_query(insert_message);
  } catch (const sql_exception & e) {
    std::cerr << "Failed to write message. Error: " << e.what() << std::endl;
    return false;
  }

  std::cout << "Stored message '" << data << "'." << std::endl;
  return true;
}

void SqliteStorage::initialize()
{
  if (is_open()) {
    try {
      std::string create_table = "CREATE TABLE messages(" \
        "id INTEGER PRIMARY KEY AUTOINCREMENT," \
        "data           BLOB    NOT NULL," \
        "timestamp      INT     NOT NULL);";

      database_->execute_query(create_table);
    } catch (const sql_exception & e) {
      std::cerr << "Could not initialize database. Error: " << e.what() << std::endl;
      database_.reset();
    }
  }

  std::cout << "Initialized database." << std::endl;
}

void SqliteStorage::open(const std::string & database_name)
{
  database_ = std::make_unique<SqliteWrapper>(database_name);
  if (!is_open()) {
    std::cerr << "Could not open database '" << database_name << "'." << std::endl;
  }

  std::cout << "Opened database '" << database_name << "'." << std::endl;
}

}  // namespace rosbag2
