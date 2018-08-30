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
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/logging_macros.h"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_storage_plugins
{
const char * ROS_PACKAGE_NAME = "rosbag2_storage_default_plugins";

SqliteStorage::SqliteStorage()
: database_(), bag_info_(), write_statement_(nullptr), read_statement_(nullptr),
  message_result_(nullptr),
  current_message_row_(nullptr, SqliteStatementWrapper::QueryResult<>::Iterator::POSITION_END)
{}

void SqliteStorage::open(
  const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  // TODO(MARTIN-IDEL-SI): use flags to open database for read-only
  (void) io_flag;
  try {
    database_ = std::make_unique<SqliteWrapper>(uri);
    bag_info_.uri = uri;
  } catch (const SqliteException & e) {
    throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
  }

  RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Opened database '%s'.", uri.c_str());
}

void SqliteStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  if (!write_statement_) {
    prepare_for_writing();
  }

  write_statement_->bind(message->serialized_data, message->time_stamp);
  write_statement_->execute_and_reset();

  RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Stored message.");
}

bool SqliteStorage::has_next()
{
  if (!read_statement_) {
    prepare_for_reading();
  }

  return current_message_row_ != message_result_.end();
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> SqliteStorage::read_next()
{
  if (!read_statement_) {
    prepare_for_reading();
  }

  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  std::tie(bag_message->serialized_data, bag_message->time_stamp) = *current_message_row_;
  ++current_message_row_;
  return bag_message;
}

void SqliteStorage::initialize()
{
  std::string create_table = "CREATE TABLE messages(" \
    "id INTEGER PRIMARY KEY AUTOINCREMENT," \
    "data           BLOB    NOT NULL," \
    "timestamp      INTEGER     NOT NULL);";

  database_->prepare_statement(create_table)->execute_and_reset();
}

rosbag2_storage::BagInfo SqliteStorage::info()
{
  return bag_info_;
}

void SqliteStorage::create_topic()
{
  initialize();
}

void SqliteStorage::prepare_for_writing()
{
  write_statement_ = database_->prepare_statement(
    "INSERT INTO messages (data, timestamp) VALUES (?, ?);");
}

void SqliteStorage::prepare_for_reading()
{
  read_statement_ =
    database_->prepare_statement("SELECT data, timestamp FROM messages ORDER BY id;");
  message_result_ = read_statement_->execute_query<
    std::shared_ptr<rcutils_char_array_t>, rcutils_time_point_value_t>();
  current_message_row_ = message_result_.begin();
}

}  // namespace rosbag2_storage_plugins


#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_storage_plugins::SqliteStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
