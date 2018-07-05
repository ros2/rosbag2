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

SqliteStorage::SqliteStorage(const std::string & database_name)
: database_(), database_name_(database_name) {}

SqliteStorage::~SqliteStorage()
{
  close();
}

bool SqliteStorage::create() 
{
  std::ifstream infile(database_name_);
  if (infile.good()) {
    std::cerr << "Database '" << database_name_ << "' already exists." << std::endl;
    return false;
  }

  try {
    database_ = sqlite::open(database_name_);
  } catch (const sqlite::sql_exception & e) {
    std::cerr << "Could not create database '" << database_name_ << "'." << std::endl;
    return false;
  }

  std::string create_table = "CREATE TABLE messages(" \
      "id INTEGER PRIMARY KEY AUTOINCREMENT," \
      "data           BLOB    NOT NULL," \
      "timestamp      INT     NOT NULL);";

  sqlite::execute_query(database_, create_table);
  std::cout << "Database '" << database_name_ << "' created" << std::endl;
  return true;
}

bool SqliteStorage::open()
{
  std::ifstream infile(database_name_);
  if (!infile.good()) {
    std::cerr << "Database '" << database_name_ << "' does not exists." << std::endl;
    return false;
  }

  try {
    database_ = sqlite::open(database_name_);
  } catch (const sqlite::sql_exception & e) {
    std::cerr << "Could not open database '" << database_name_ << "'." << std::endl;
    return false;
  }

  std::cout << "Database '" << database_name_ << "' openend" << std::endl;
  return true;
}

bool SqliteStorage::write(const std::string & data)
{
  if (!database_) {
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

void SqliteStorage::close()
{
  if (database_) {
    sqlite::close(database_);
  }
}

sqlite::DBPtr SqliteStorage::getDatabaseHandle()
{
  try {
    database_ = sqlite::open(database_name_);
  } catch (const sqlite::sql_exception & e) {
    std::cerr << "Could not open database '" << database_name_ << "'." << std::endl;
    return nullptr;
  }
  return database_;
}

}  // namespace rosbag2
