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

#include <fstream>
#include <iostream>
#include <string>

namespace rosbag2
{

SqliteStorage::SqliteStorage(const std::string & database_name, bool open_for_writing)
: database_()
{
  if (open_for_writing) {
    create_and_open(database_name);
  } else {
    open(database_name);
  }
}

SqliteStorage::~SqliteStorage()
{
  if (isOpen()) {
    sqlite::close(database_);
  }
}

bool SqliteStorage::isOpen()
{
  return static_cast<bool>(database_);
}

bool SqliteStorage::write(const std::string & data)
{
  if (!isOpen()) {
    std::cerr << "Failed to write message. The database is not open." << std::endl;
    return false;
  }

  try {
    std::string insert_message =
      "INSERT INTO messages (data, timestamp) VALUES ('" + data + "', strftime('%s%f','now'))";
    sqlite::execute_query(database_, insert_message);
  } catch (const sqlite::sql_exception & e) {
    std::cerr << "Failed to write message. Error: " << e.what() << std::endl;
    return false;
  }

  std::cout << "Stored message '" << data << "'." << std::endl;
  return true;
}

void SqliteStorage::create_and_open(const std::string & database_name)
{
  std::ifstream infile(database_name);
  if (infile.good()) {
    std::cerr << "Database '" << database_name << "' already exists." << std::endl;
    return;
  }

  try {
    database_ = sqlite::open(database_name);
    std::string create_table = "CREATE TABLE messages(" \
      "id INTEGER PRIMARY KEY AUTOINCREMENT," \
      "data           BLOB    NOT NULL," \
      "timestamp      INT     NOT NULL);";

    sqlite::execute_query(database_, create_table);
  } catch (const sqlite::sql_exception & e) {
    std::cerr << "Could not create database '" << database_name << "'. " <<
      "Error: " << e.what() << std::endl;
    database_ = nullptr;
    return;
  }

  std::cout << "Database '" << database_name << "' created and opened" << std::endl;
}

void SqliteStorage::open(const std::string & database_name)
{
  std::ifstream infile(database_name);
  if (!infile.good()) {
    std::cerr << "Database '" << database_name << "' does not exists." << std::endl;
    return;
  }

  try {
    database_ = sqlite::open(database_name);
  } catch (const sqlite::sql_exception & e) {
    std::cerr << "Could not open database '" << database_name << "'." <<
      "Error: " << e.what() << std::endl;
    return;
  }

  std::cout << "Database '" << database_name << "' opened" << std::endl;
}

}  // namespace rosbag2
