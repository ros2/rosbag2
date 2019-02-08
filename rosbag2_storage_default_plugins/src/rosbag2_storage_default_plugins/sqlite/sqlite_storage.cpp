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
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include "rosbag2_storage/filesystem_helper.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_statement_wrapper.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_exception.hpp"
#include "rosbag2_storage/ros_helper.hpp"

#include "../logging.hpp"

namespace rosbag2_storage_plugins
{

SqliteStorage::SqliteStorage()
: database_(),
  write_statement_(nullptr),
  read_statement_(nullptr),
  message_result_(nullptr),
  current_message_row_(nullptr, SqliteStatementWrapper::QueryResult<>::Iterator::POSITION_END),
  extra_file_(true)
{}

void SqliteStorage::open(
  const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  auto metadata = is_read_only(io_flag) ?
    load_metadata(uri) :
    std::unique_ptr<rosbag2_storage::BagMetadata>();

  if (metadata) {
    if (metadata->relative_file_paths.empty()) {
      throw std::runtime_error(
              "Failed to read from bag '" + uri + "': Missing database file path in metadata");
    }

    database_name_ = metadata->relative_file_paths[0];
  } else {
    if (is_read_only(io_flag)) {
      throw std::runtime_error("Failed to read from bag '" + uri + "': No metadata found.");
    }

    database_name_ = rosbag2_storage::FilesystemHelper::get_folder_name(uri) + ".db3";
  }

  std::string database_path = rosbag2_storage::FilesystemHelper::concat({uri, database_name_});
  if (is_read_only(io_flag) && !database_exists(database_path)) {
    throw std::runtime_error(
            "Failed to read from bag '" + uri + "': File '" + database_name_ + "' does not exist.");
  }

  try {
    database_ = std::make_unique<SqliteWrapper>(database_path, io_flag);
  } catch (const SqliteException & e) {
    throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
  }

  if (!metadata) {
    initialize();
  }

  if (extra_file_) {
    const bool ro = is_read_only(io_flag);

    // in write mode we add at least one serialized file
    // (it would be possible to split dante into multiple files)
    if(!ro)
    {
      data_files_.push_back(
        std::make_unique<BinarySequentialFile>(database_name_ + ".extra")
      );
    }
    else if(metadata->relative_file_paths.size()>=1)
    {
      std::for_each(metadata->relative_file_paths.begin()+1, metadata->relative_file_paths.end(), [this](auto &path)
      {
        data_files_.push_back(
          std::make_unique<BinarySequentialFile>(path)
        );
      });
    }

    // no serialized file, fall back to old mode
    if(data_files_.size()==0)
    {
      extra_file_ = false;
    }
    else
    {
      for(auto &data_file : data_files_)
      {
        data_file->file_stream.open(
          rosbag2_storage::FilesystemHelper::concat({uri, data_file->path}),
          std::fstream::binary | (ro ? std::fstream::in : std::fstream::out)
        );

        if (!data_file->file_stream)
        {
          throw std::runtime_error("Failed to setup storage. Failed to open extra file!");
        }
      }
    }

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

  if (extra_file_) {
    constexpr uint32_t file_index = 0;  // could be changed to split data into files

    auto &file_handle = data_files_[file_index];
    std::lock_guard<std::mutex>(file_handle->lock);

    MetaInformation data = {
      static_cast<uint64_t>(file_handle->file_stream.tellg()),
      static_cast<uint64_t>(message->serialized_data->buffer_length),
      file_index
    };

    file_handle->file_stream.write(reinterpret_cast<const char *>(message->serialized_data->buffer),
      message->serialized_data->buffer_length);

    auto serialized_data = std::make_shared<rcutils_uint8_array_t>();
    serialized_data->buffer = reinterpret_cast<uint8_t *>(&data);
    serialized_data->buffer_length = serialized_data->buffer_capacity = sizeof(MetaInformation);

    write_statement_->bind(message->time_stamp, topic_entry->second, serialized_data);
    write_statement_->execute_and_reset();

    serialized_data->buffer = nullptr;
    serialized_data->buffer_length = serialized_data->buffer_capacity = 0;
  } else {
    write_statement_->bind(message->time_stamp, topic_entry->second, message->serialized_data);
    write_statement_->execute_and_reset();
  }
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
  if (extra_file_) {
    MetaInformation *data;
    std::shared_ptr<rcutils_uint8_array_t> serialized_data = std::get<0>(*current_message_row_);
    if (serialized_data->buffer_length == sizeof(MetaInformation)) {

      data = reinterpret_cast<MetaInformation *>(serialized_data->buffer);

      if(!data || data->file_index>=data_files_.size())
      {
        throw std::runtime_error("Invalid content for extra file!");
      }

      auto &file_handle = data_files_[data->file_index];
      std::lock_guard<std::mutex>(file_handle->lock);

      bag_message->serialized_data = rosbag2_storage::make_empty_serialized_message(data->size);
      bag_message->serialized_data->buffer_length = data->size;
      file_handle->file_stream.seekg(data->offset);

      file_handle->file_stream.read(reinterpret_cast<char *>(
        bag_message->serialized_data->buffer),
        bag_message->serialized_data->buffer_length
      );
    } else {
      throw std::runtime_error("Database entry has wrong size!");
    }
  } else {
    bag_message->serialized_data = std::get<0>(*current_message_row_);
  }
  bag_message->time_stamp = std::get<1>(*current_message_row_);
  bag_message->topic_name = std::get<2>(*current_message_row_);

  ++current_message_row_;
  return bag_message;
}

std::vector<rosbag2_storage::TopicMetadata> SqliteStorage::get_all_topics_and_types()
{
  if (all_topics_and_types_.empty()) {
    fill_topics_and_types();
  }

  return all_topics_and_types_;
}

void SqliteStorage::initialize()
{
  std::string create_stmt = "CREATE TABLE topics(" \
    "id INTEGER PRIMARY KEY," \
    "name TEXT NOT NULL," \
    "type TEXT NOT NULL," \
    "serialization_format TEXT NOT NULL);";
  database_->prepare_statement(create_stmt)->execute_and_reset();
  create_stmt = "CREATE TABLE messages(" \
    "id INTEGER PRIMARY KEY," \
    "topic_id INTEGER NOT NULL," \
    "timestamp INTEGER NOT NULL, " \
    "data BLOB NOT NULL);";
  database_->prepare_statement(create_stmt)->execute_and_reset();
  create_stmt = "CREATE INDEX timestamp_idx ON messages (timestamp ASC);";
  database_->prepare_statement(create_stmt)->execute_and_reset();
}

void SqliteStorage::create_topic(const rosbag2_storage::TopicMetadata & topic)
{
  if (topics_.find(topic.name) == std::end(topics_)) {
    auto insert_topic =
      database_->prepare_statement(
      "INSERT INTO topics (name, type, serialization_format) VALUES (?, ?, ?)");
    insert_topic->bind(topic.name, topic.type, topic.serialization_format);
    insert_topic->execute_and_reset();
    topics_.emplace(topic.name, static_cast<int>(database_->get_last_insert_id()));
  }
}

void SqliteStorage::remove_topic(const rosbag2_storage::TopicMetadata & topic)
{
  if (topics_.find(topic.name) != std::end(topics_)) {
    auto delete_topic =
      database_->prepare_statement(
      "DELETE FROM topics where name = ? and type = ? and serialization_format = ?");
    delete_topic->bind(topic.name, topic.type, topic.serialization_format);
    delete_topic->execute_and_reset();
    topics_.erase(topic.name);
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
    std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string>();
  current_message_row_ = message_result_.begin();
}

void SqliteStorage::fill_topics_and_types()
{
  auto statement = database_->prepare_statement(
    "SELECT name, type, serialization_format FROM topics ORDER BY id;");
  auto query_results = statement->execute_query<std::string, std::string, std::string>();

  for (auto result : query_results) {
    all_topics_and_types_.push_back(
      {std::get<0>(result), std::get<1>(result), std::get<2>(result)});
  }
}

std::unique_ptr<rosbag2_storage::BagMetadata> SqliteStorage::load_metadata(const std::string & uri)
{
  try {
    rosbag2_storage::MetadataIo metadata_io;
    return std::make_unique<rosbag2_storage::BagMetadata>(metadata_io.read_metadata(uri));
  } catch (std::exception & e) {
    ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_ERROR("Failed to load metadata: %s", e.what());
    return std::unique_ptr<rosbag2_storage::BagMetadata>();
  }
}

bool SqliteStorage::database_exists(const std::string & uri)
{
  std::ifstream database(uri);
  return database.good();
}

bool SqliteStorage::is_read_only(const rosbag2_storage::storage_interfaces::IOFlag & io_flag) const
{
  return io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY;
}

rosbag2_storage::BagMetadata SqliteStorage::get_metadata()
{
  rosbag2_storage::BagMetadata metadata;
  metadata.storage_identifier = "sqlite3";
  metadata.relative_file_paths = {database_name_};
  for(const auto &f : data_files_)
  {
    metadata.relative_file_paths.push_back(f->path);
  }

  metadata.message_count = 0;
  metadata.topics_with_message_count = {};

  auto statement = database_->prepare_statement(
    "SELECT name, type, serialization_format, COUNT(messages.id), MIN(messages.timestamp), "
    "MAX(messages.timestamp) "
    "FROM messages JOIN topics on topics.id = messages.topic_id "
    "GROUP BY topics.name;");
  auto query_results = statement->execute_query<
    std::string, std::string, std::string, int, rcutils_time_point_value_t,
    rcutils_time_point_value_t>();

  rcutils_time_point_value_t min_time = INT64_MAX;
  rcutils_time_point_value_t max_time = 0;
  for (auto result : query_results) {
    metadata.topics_with_message_count.push_back(
      {
        {std::get<0>(result), std::get<1>(result), std::get<2>(result)},
        static_cast<size_t>(std::get<3>(result))
      });

    metadata.message_count += std::get<3>(result);
    min_time = std::get<4>(result) < min_time ? std::get<4>(result) : min_time;
    max_time = std::get<5>(result) > max_time ? std::get<5>(result) : max_time;
  }

  if (metadata.message_count == 0) {
    min_time = 0;
    max_time = 0;
  }

  metadata.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(min_time));
  metadata.duration = std::chrono::nanoseconds(max_time) - std::chrono::nanoseconds(min_time);
  metadata.bag_size = rosbag2_storage::FilesystemHelper::calculate_directory_size(database_name_);

  return metadata;
}

}  // namespace rosbag2_storage_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_storage_plugins::SqliteStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
