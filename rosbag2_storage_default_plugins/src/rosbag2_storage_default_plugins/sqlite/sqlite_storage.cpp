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

#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"

#include <sys/stat.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_storage/filesystem_helpers.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_statement_wrapper.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_exception.hpp"

#include "../logging.hpp"

namespace rosbag2_storage_plugins
{

SqliteStorage::SqliteStorage()
: database_(),
  metadata_(),
  write_statement_(nullptr),
  read_statement_(nullptr),
  message_result_(nullptr),
  current_message_row_(nullptr, SqliteStatementWrapper::QueryResult<>::Iterator::POSITION_END)
{
  metadata_.storage_identifier = "sqlite3";
  metadata_.encoding = "cdr";  // TODO(greimela) Determine encoding
}

void SqliteStorage::open(
  const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  std::string database_name = get_database_name(uri);
  std::string database_path = uri + rosbag2_storage::separator() + database_name;

  if (io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY &&
    !database_exists(database_path))
  {
    throw std::runtime_error("Failed to read from bag '" + uri + "': Folder does not exist.");
  }

  try {
    database_ = std::make_unique<SqliteWrapper>(database_path, io_flag);
    metadata_.relative_file_paths = {database_name};
  } catch (const SqliteException & e) {
    throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
  }

  if (io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) {
    initialize();
  }

  ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_INFO_STREAM("Opened database '" << uri << "'.");
}

void SqliteStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  if (!write_statement_) {
    prepare_for_writing();
  }
  auto topic_entry = topics_.find(message->topic_name);
  if (topic_entry == end(topics_)) {
    throw SqliteException("Topic '" + message->topic_name +
            "' has not been created yet! Call 'create_topic' first.");
  }

  write_statement_->bind(message->time_stamp, topic_entry->second, message->serialized_data);
  write_statement_->execute_and_reset();
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
  bag_message->serialized_data = std::get<0>(*current_message_row_);
  bag_message->time_stamp = std::get<1>(*current_message_row_);
  bag_message->topic_name = std::get<2>(*current_message_row_);

  ++current_message_row_;
  return bag_message;
}

std::vector<rosbag2_storage::TopicWithType> SqliteStorage::get_all_topics_and_types()
{
  if (all_topics_and_types_.empty()) {
    fill_topics_and_types();
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

void SqliteStorage::create_topic(const rosbag2_storage::TopicWithType & topic)
{
  if (topics_.find(topic.name) == std::end(topics_)) {
    auto insert_topic =
      database_->prepare_statement("INSERT INTO topics (name, type) VALUES (?, ?)");
    insert_topic->bind(topic.name, topic.type);
    insert_topic->execute_and_reset();
    topics_.emplace(topic.name, static_cast<int>(database_->get_last_insert_id()));
  }
}

void SqliteStorage::prepare_for_writing()
{
  write_statement_ = database_->prepare_statement(
    "INSERT INTO messages (timestamp, topic_id, data) VALUES (?, ?, ?);");
}

void SqliteStorage::prepare_for_reading()
{
  read_statement_ = database_->prepare_statement(
    "SELECT data, timestamp, topics.name "
    "FROM messages JOIN topics ON messages.topic_id = topics.id "
    "ORDER BY messages.timestamp;");
  message_result_ = read_statement_->execute_query<
    std::shared_ptr<rcutils_char_array_t>, rcutils_time_point_value_t, std::string>();
  current_message_row_ = message_result_.begin();
}

void SqliteStorage::fill_topics_and_types()
{
  auto statement = database_->prepare_statement("SELECT name, type FROM topics ORDER BY id;");
  auto query_results = statement->execute_query<std::string, std::string>();

  for (auto result : query_results) {
    all_topics_and_types_.push_back({std::get<0>(result), std::get<1>(result)});
  }
}

std::string SqliteStorage::get_database_name(const std::string & uri)
{
  return rosbag2_storage::get_leaf_directory(uri) + ".db3";
}

bool SqliteStorage::database_exists(const std::string & uri)
{
  std::ifstream database(uri);
  return database.good();
}

rosbag2_storage::BagMetadata SqliteStorage::get_metadata()
{
  metadata_.message_count = 0;
  metadata_.topics_with_message_count = {};

  auto statement = database_->prepare_statement(
    "SELECT name, type, COUNT(messages.id), MIN(messages.timestamp), MAX(messages.timestamp) "
    "FROM messages JOIN topics on topics.id = messages.topic_id "
    "GROUP BY topics.name;");
  auto query_results = statement->execute_query<
    std::string, std::string, int, rcutils_time_point_value_t, rcutils_time_point_value_t>();

  rcutils_time_point_value_t min_time = INT64_MAX;
  rcutils_time_point_value_t max_time = 0;
  for (auto result : query_results) {
    metadata_.topics_with_message_count.push_back(
      {
        {std::get<0>(result), std::get<1>(result)},
        static_cast<size_t>(std::get<2>(result))
      });
    metadata_.message_count += std::get<2>(result);
    min_time = std::get<3>(result) < min_time ? std::get<3>(result) : min_time;
    max_time = std::get<4>(result) > max_time ? std::get<4>(result) : max_time;
  }
  metadata_.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(min_time));
  metadata_.duration = std::chrono::nanoseconds(max_time) - std::chrono::nanoseconds(min_time);

  return metadata_;
}

}  // namespace rosbag2_storage_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_storage_plugins::SqliteStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
