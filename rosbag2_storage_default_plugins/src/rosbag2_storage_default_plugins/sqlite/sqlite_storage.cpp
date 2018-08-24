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
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/logging_macros.h"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "sqlite_statement_wrapper.hpp"

namespace rosbag2_storage_plugins
{
const char * ROS_PACKAGE_NAME = "rosbag2_storage_default_plugins";

SqliteStorage::SqliteStorage()
: database_(),
  bag_info_(),
  write_statement_(nullptr),
  read_statement_(nullptr),
  message_result_(nullptr),
  current_message_row_(nullptr, SqliteStatementWrapper::QueryResult<>::Iterator::POSITION_END)
{}

void SqliteStorage::open(
  const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  if (io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY && !database_exists(uri)) {
    throw std::runtime_error("Failed to read from bag '" + uri + "': file does not exist.");
  }

  try {
    database_ = std::make_unique<SqliteWrapper>(uri);
    bag_info_.uri = uri;
  } catch (const SqliteException & e) {
    throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
  }

  if (io_flag != rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    initialize();
  }

  RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Opened database '%s'.", uri.c_str());
}

void SqliteStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  if (!write_statement_) {
    prepare_for_writing();
  }
  auto topic_entry = topics_.find(message->topic_name);
  if (topic_entry == end(topics_)) {
    throw SqliteStorageException("Topic '" + message->topic_name +
            "' has not been created yet! Call 'create_topic' first.");
  }

  write_statement_->bind(message->time_stamp, topic_entry->second, message->serialized_data);
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

std::map<std::string, std::string> SqliteStorage::get_all_topics_and_types()
{
  if (all_topics_and_types_.empty()) {
    fill_topics_and_types_map();
  }

  return all_topics_and_types_;
}

void SqliteStorage::initialize()
{
  std::string create_table = "CREATE TABLE topics(" \
    "id INTEGER PRIMARY KEY," \
    "name TEXT NOT NULL," \
    "type TEXT NOT NULL);";
  database_->prepare_statement(create_table)->execute_and_reset();
  create_table = "CREATE TABLE messages(" \
    "id INTEGER PRIMARY KEY," \
    "topic_id INTEGER NOT NULL," \
    "timestamp INTEGER NOT NULL, " \
    "data BLOB NOT NULL);";
  database_->prepare_statement(create_table)->execute_and_reset();
}

rosbag2_storage::BagInfo SqliteStorage::info()
{
  return bag_info_;
}

void SqliteStorage::create_topic(const std::string & name, const std::string & type_id)
{
  if (topics_.find(name) == std::end(topics_)) {
    auto insert_topic =
      database_->prepare_statement("INSERT INTO topics (name, type) VALUES (?, ?)");
    insert_topic->bind(name, type_id);
    insert_topic->execute_and_reset();
    topics_.emplace(name, static_cast<int>(database_->get_last_insert_id()));
  }
}

void SqliteStorage::prepare_for_writing()
{
  write_statement_ = database_->prepare_statement(
    "INSERT INTO messages (timestamp, topic_id, data) VALUES (?, ?, ?);");
}

void SqliteStorage::prepare_for_reading()
{
  read_statement_ =
    database_->prepare_statement("SELECT data, timestamp FROM messages ORDER BY id;");
  message_result_ = read_statement_->execute_query<
    std::shared_ptr<rcutils_char_array_t>, rcutils_time_point_value_t>();
  current_message_row_ = message_result_.begin();
}

void SqliteStorage::fill_topics_and_types_map()
{
  auto statement = database_->prepare_statement("SELECT name, type FROM topics ORDER BY id;");
  auto query_results = statement->execute_query<std::string, std::string>();

  for (auto result : query_results) {
    all_topics_and_types_.insert(std::make_pair(std::get<0>(result), std::get<1>(result)));
  }
}

bool SqliteStorage::database_exists(const std::string & uri)
{
  std::ifstream database(uri);
  return database.good();
}

}  // namespace rosbag2_storage_plugins


#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_storage_plugins::SqliteStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
