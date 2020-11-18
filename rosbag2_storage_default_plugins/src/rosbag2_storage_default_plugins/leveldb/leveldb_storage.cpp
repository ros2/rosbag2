// Copyright 2020 Sony Corporation
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

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage_default_plugins/leveldb/leveldb_exception.hpp"

#include "rosbag2_storage_default_plugins/leveldb/leveldb_storage.hpp"

#include "../logging.hpp"

namespace
{
std::string to_string(rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  switch (io_flag) {
    case rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY:
      return "READ_ONLY";
    case rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE:
      return "READ_WRITE";
    case rosbag2_storage::storage_interfaces::IOFlag::APPEND:
      return "APPEND";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

namespace rosbag2_storage_plugins
{

LeveldbStorage::~LeveldbStorage()
{
}

void LeveldbStorage::open(
  const rosbag2_storage::StorageOptions & storage_options,
  rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  relative_path_ = storage_options.uri;

  rcpputils::fs::path path(relative_path_);

  if (io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) {
    // READ_WRITE requires the DB to not exist.
    if (path.exists()) {
      throw std::runtime_error(
              "Failed to create bag: File '" + relative_path_ + "' already exists!");
    }

    bool ret = rcpputils::fs::create_directories(relative_path_);
    if (!ret) {
      throw std::runtime_error("Failed to create directory " + relative_path_);
    }
  } else {  // APPEND and READ_ONLY
    // APPEND and READ_ONLY require the DB to exist
    if (!path.exists()) {
      throw std::runtime_error(
              "Failed to read from bag: File '" + relative_path_ + "' does not exist!");
    }
  }

  ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_INFO_STREAM(
    "Opened database '" << relative_path_ << "' for " << to_string(io_flag) << ".");
}

void LeveldbStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  auto topic_ldb = topic_ldb_map_.find(message->topic_name);
  if (topic_ldb == topic_ldb_map_.end()) {
    throw LeveldbException(
            "Topic '" + message->topic_name +
            "' has not been created yet! Call 'create_topic' first.");
  }

  topic_ldb->second->write_message(message);
}

void LeveldbStorage::write(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & messages)
{
  for (auto msg : messages) {
    // Since message may be from different topic, have to do one by one.
    write(msg);
  }
}

inline void LeveldbStorage::scan_ldb_for_read()
{
  if (scan_ldb_done_) {
    return;
  }

#ifdef _WIN32
  WIN32_FIND_DATA data;
  rcpputils::fs::path dir_path(relative_path_ + "\\*");
  HANDLE handle = FindFirstFile(dir_path.string().c_str(), &data);
  if (INVALID_HANDLE_VALUE == handle) {
    throw std::runtime_error("Can't open directory " + relative_path_ + " !");
  }

  do {
    // Skip over local folder handle (`.`) and parent folder (`..`)
    if (strcmp(data.cFileName, ".") != 0 && strcmp(data.cFileName, "..") != 0) {
      rcpputils::fs::path path(relative_path_ + "\\" + data.cFileName);
      if (path.is_directory()) {
        std::string dir_name(data.cFileName);
        // Leveldb directory name format : ${TOPIC_NAME}_${LDB_METADATA_POSTFIX}
        if (dir_name.find(LDB_METADATA_POSTFIX) ==
          (dir_name.length() - strlen(LDB_METADATA_POSTFIX)))
        {
          std::string base_dir_name =
            dir_name.substr(0, dir_name.length() - strlen(LDB_METADATA_POSTFIX));
          std::shared_ptr<class LeveldbWrapper> ldb_wrapper =
            std::make_shared<class LeveldbWrapper>(relative_path_, "", base_dir_name, false);
          ldb_wrapper->init_ldb();
          topic_ldb_map_.emplace(std::make_pair(ldb_wrapper->get_topic_name(), ldb_wrapper));
        }
      }
    }
  } while (FindNextFile(handle, &data));
  FindClose(handle);
#else
  DIR * dir = opendir(relative_path_.c_str());
  if (nullptr == dir) {
    throw std::runtime_error("Can't open directory " + relative_path_ + " !");
  }

  // Scan path to find how many topic exist
  struct dirent * directory_entry;
  while ((directory_entry = readdir(dir)) != nullptr) {
    if (strcmp(directory_entry->d_name, ".") != 0 && strcmp(directory_entry->d_name, "..") != 0) {
      rcpputils::fs::path path(relative_path_ + "/" + directory_entry->d_name);
      if (path.is_directory()) {
        std::string dir_name(directory_entry->d_name);
        // Leveldb directory name format : ${TOPIC_NAME}_${LDB_METADATA_POSTFIX}
        if (dir_name.find(LDB_METADATA_POSTFIX) ==
          (dir_name.length() - strlen(LDB_METADATA_POSTFIX)))
        {
          std::string base_dir_name =
            dir_name.substr(0, dir_name.length() - strlen(LDB_METADATA_POSTFIX));
          std::shared_ptr<class LeveldbWrapper> ldb_wrapper =
            std::make_shared<class LeveldbWrapper>(relative_path_, "", base_dir_name, false);
          ldb_wrapper->init_ldb();
          topic_ldb_map_.emplace(std::make_pair(ldb_wrapper->get_topic_name(), ldb_wrapper));
        }
      }
    }
  }
#endif

  if (topic_ldb_map_.empty()) {
    ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_WARN_STREAM(
      "Not find leveldb database in " << relative_path_);
  }

  scan_ldb_done_ = true;
}


bool LeveldbStorage::has_next()
{
  scan_ldb_for_read();

  return !read_cache_.empty();
}


std::shared_ptr<rosbag2_storage::SerializedBagMessage> LeveldbStorage::read_next()
{
  scan_ldb_for_read();

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message;

  if (!read_cache_.empty()) {
    // message in read_cache_ has been ordered by timestamp
    auto iter = read_cache_.begin();
    bag_message = iter->second;

    // Remove read one
    read_cache_.erase(iter);

    // Which topic consume message, read new one.
    if (topic_ldb_map_[bag_message->topic_name]->has_next()) {
      std::shared_ptr<rosbag2_storage::SerializedBagMessage> new_msg =
        topic_ldb_map_[bag_message->topic_name]->read_next();

      read_cache_.emplace(
        std::pair<rcutils_time_point_value_t,
        std::shared_ptr<rosbag2_storage::SerializedBagMessage>>(new_msg->time_stamp, new_msg));
    }
  }

  return bag_message;
}

std::vector<rosbag2_storage::TopicMetadata> LeveldbStorage::get_all_topics_and_types()
{
  if (all_topics_and_types_.empty()) {
    fill_topics_and_types();
  }

  return all_topics_and_types_;
}

uint64_t LeveldbStorage::get_bagfile_size() const
{
  uint64_t summary_size = 0;
  for (auto topic_ldb : topic_ldb_map_) {
    summary_size += topic_ldb.second->get_topic_ldb_size();
  }

  return summary_size;
}

void LeveldbStorage::create_topic(const rosbag2_storage::TopicMetadata & topic)
{
  if (topic_ldb_map_.find(topic.name) == topic_ldb_map_.end()) {
    // The "/" in topic name will be replaced by '_' for directory name
    // e.g. /ns1/topic => _ns1_topic
    std::string dir_name = topic.name;
    std::replace(dir_name.begin(), dir_name.end(), '/', '_');
    std::shared_ptr<class LeveldbWrapper> ldb_wrapper =
      std::make_shared<class LeveldbWrapper>(relative_path_, topic.name, dir_name, true);


    ldb_wrapper->init_ldb();
    ldb_wrapper->write_metadata(topic);

    topic_ldb_map_.emplace(std::make_pair(topic.name, std::move(ldb_wrapper)));
  }
}

void LeveldbStorage::remove_topic(const rosbag2_storage::TopicMetadata & topic)
{
  if (topic_ldb_map_.find(topic.name) != topic_ldb_map_.end()) {
    topic_ldb_map_[topic.name]->remove_database();
    topic_ldb_map_.erase(topic.name);
  }
}

void LeveldbStorage::fill_topics_and_types()
{
  for (auto topic_ldb : topic_ldb_map_) {
    std::shared_ptr<rosbag2_storage::TopicMetadata> topic_metadata =
      topic_ldb.second->read_all_metadata();
    all_topics_and_types_.emplace_back(*topic_metadata);
  }
}

std::string LeveldbStorage::get_storage_identifier() const
{
  return "leveldb";
}

std::string LeveldbStorage::get_relative_file_path() const
{
  return relative_path_;
}


uint64_t LeveldbStorage::get_minimum_split_file_size() const
{
  return leveldb::Options().max_file_size;
}

rosbag2_storage::BagMetadata LeveldbStorage::get_metadata()
{
  rosbag2_storage::BagMetadata metadata;
  metadata.storage_identifier = get_storage_identifier();
  metadata.relative_file_paths = {get_relative_file_path()};

  metadata.message_count = 0;
  metadata.topics_with_message_count = {};

  rcutils_time_point_value_t min_time = INT64_MAX;
  rcutils_time_point_value_t max_time = 0;

  // While play, metadata/message leveldb isn't really opened.
  if (topic_ldb_map_.empty()) {
    scan_ldb_for_read();
  }

  for (auto topic_ldb : topic_ldb_map_) {
    size_t msg_count = topic_ldb.second->get_message_count();
    rosbag2_storage::TopicMetadata topic_metadata = *(topic_ldb.second->read_all_metadata());
    metadata.topics_with_message_count.push_back({topic_metadata, msg_count});

    rcutils_time_point_value_t topic_min = topic_ldb.second->get_min_timestamp();
    min_time = topic_min < min_time ? topic_min : min_time;

    rcutils_time_point_value_t topic_max = topic_ldb.second->get_max_timestamp();
    max_time = topic_max > max_time ? topic_max : max_time;

    metadata.message_count += msg_count;
  }

  if (metadata.message_count == 0) {
    min_time = 0;
    max_time = 0;
  }

  metadata.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(min_time));
  metadata.duration = std::chrono::nanoseconds(max_time) - std::chrono::nanoseconds(min_time);
  metadata.bag_size = get_bagfile_size();

  return metadata;
}

void LeveldbStorage::set_filter(
  const rosbag2_storage::StorageFilter & storage_filter)
{
  storage_filter_ = storage_filter;
  std::vector<std::string> remove_topic_ldb_list;

  scan_ldb_for_read();

  if (!storage_filter_.topics.empty()) {
    // Find unfiltered topic
    for (auto topic_ldb : topic_ldb_map_) {
      bool is_found = false;
      for (auto topic_name : storage_filter.topics) {
        if (topic_ldb.first == topic_name) {
          is_found = true;
          break;
        }
      }
      if (!is_found) {
        remove_topic_ldb_list.emplace_back(topic_ldb.first);
      }
    }

    // Remove unfiltered topic
    for (auto topic_name : remove_topic_ldb_list) {
      auto iter = topic_ldb_map_.find(topic_name);
      topic_ldb_map_.erase(iter);
    }
  }

  // Read first message from each topic and put to read_cache_
  for (auto topic_ldb : topic_ldb_map_) {
    if (topic_ldb.second->has_next()) {
      std::shared_ptr<rosbag2_storage::SerializedBagMessage> message =
        topic_ldb.second->read_next();
      read_cache_.insert(
        std::pair<rcutils_time_point_value_t,
        std::shared_ptr<rosbag2_storage::SerializedBagMessage>>(message->time_stamp, message));
    }
  }
}

void LeveldbStorage::reset_filter()
{
  storage_filter_ = rosbag2_storage::StorageFilter();
}

}  // namespace rosbag2_storage_plugins


#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_storage_plugins::LeveldbStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
