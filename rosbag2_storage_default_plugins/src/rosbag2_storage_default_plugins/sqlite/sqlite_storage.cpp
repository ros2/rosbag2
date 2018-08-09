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

#include "sqlite_storage.hpp"

#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/logging_macros.h"

namespace rosbag2_storage_plugins
{
const char * ROS_PACKAGE_NAME = "rosbag2_storage_default_plugins";

SqliteStorage::SqliteStorage()
: database_(), counter_(0), bag_info_()
{}

SqliteStorage::SqliteStorage(std::shared_ptr<SqliteWrapper> database)
: database_(std::move(database)), counter_(0)
{}

void SqliteStorage::open(const std::string & uri)
{
  try {
    database_ = std::make_unique<SqliteWrapper>(uri);
    bag_info_.uri = uri;
  } catch (const SqliteException & e) {
    throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
  }

  RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Opened database '%s'.", uri.c_str());
}

void SqliteStorage::open_readonly(const std::string & uri)
{
  try {
    database_ = std::make_unique<SqliteWrapper>(uri);
    bag_info_.uri = uri;
  } catch (const SqliteException & e) {
    throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
  }

  RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Opened database '%s'.", uri.c_str());
}

void SqliteStorage::write(std::string message)
{
  std::string insert_message =
    "INSERT INTO messages (data, timestamp) VALUES ('" + message + "', strftime('%s%f','now'))";
  database_->execute_query(insert_message);

  RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Stored message '%s'.", message.c_str());
}

bool SqliteStorage::has_next()
{
  // TODO(Martin-Idel-SI): improve sqlite_wrapper interface
  std::string message;
  return database_->get_message(message, counter_);
}

std::string SqliteStorage::read_next()
{
  // TODO(Martin-Idel-SI): improve sqlite_wrapper interface
  std::string message;
  database_->get_message(message, counter_++);
  return message;
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
  return bag_info_;
}

void SqliteStorage::create_topic()
{
  initialize();
}

}  // namespace rosbag2_storage_plugins


#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_storage_plugins::SqliteStorage, rosbag2_storage::ReadWriteStorage)
