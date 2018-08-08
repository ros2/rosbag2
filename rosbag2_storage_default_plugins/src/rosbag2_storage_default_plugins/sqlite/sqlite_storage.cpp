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

#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rosbag2_storage_plugins
{
SqliteStorage::SqliteStorage()
: database_(), counter_(0)
{}

SqliteStorage::SqliteStorage(std::shared_ptr<SqliteWrapper> database)
: database_(std::move(database)), counter_(0)
{}

void SqliteStorage::open_for_reading(const std::string & uri)
{
  try {
    database_ = std::make_unique<SqliteWrapper>(uri);
  } catch (const SqliteException & e) {
    throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
  }

  std::cout << "Opened database '" << uri << "'." << std::endl;
}

void SqliteStorage::open_for_writing(const std::string & uri)
{
  try {
    database_ = std::make_unique<SqliteWrapper>(uri);
    initialize();
  } catch (const SqliteException & e) {
    throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
  }

  std::cout << "Opened database '" << uri << "'." << std::endl;
}

bool SqliteStorage::write(void * data, size_t size)
{
  (void) size;
  auto message = static_cast<std::string *>(data);
  try {
    std::string insert_message =
      "INSERT INTO messages (data, timestamp) VALUES ('" + *message +
      "', strftime('%s%f','now'))";
    database_->execute_query(insert_message);
  } catch (const SqliteException & e) {
    std::cerr << "Failed to write message. Error: " << e.what() << std::endl;
    return false;
  }

  std::cout << "Stored message '" << *message << "'." << std::endl;
  return true;
}

bool SqliteStorage::read_next(void * buffer, size_t & size)
{
  try {
    return database_->get_message(buffer, size, counter_++);
  } catch (const SqliteException & e) {
    std::cerr << "Failed to read messages. Error: " << e.what() << std::endl;
    return false;
  }
}

void SqliteStorage::initialize()
{
  std::string create_table = "CREATE TABLE messages(" \
    "id INTEGER PRIMARY KEY AUTOINCREMENT," \
    "data           BLOB    NOT NULL," \
    "timestamp      INT     NOT NULL);";

  database_->execute_query(create_table);
}

rosbag2_storage::BagInfo SqliteStorage::info()
{
  return rosbag2_storage::BagInfo();
}

}  // namespace rosbag2_storage_plugins


#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_storage_plugins::SqliteStorage, rosbag2_storage::WritableStorage)
PLUGINLIB_EXPORT_CLASS(rosbag2_storage_plugins::SqliteStorage, rosbag2_storage::ReadableStorage)
