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

#include <sys/stat.h>
#include <sys/types.h>

#include <string>
#include <utility>
#include <memory>
#include <vector>

#include "rosbag2_storage_default_plugins/leveldb/leveldb_wrapper.hpp"

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/filesystem.h"
#include "rosbag2_storage/ros_helper.hpp"

#include "../logging.hpp"

namespace rosbag2_storage_plugins
{

LeveldbWrapper::LeveldbWrapper(
  const std::string relative_path,
  const std::string topic_name,
  const std::string dir_name,
  bool readwrite)
: path_metadata_(relative_path + "/" + dir_name + LDB_METADATA_POSTFIX),
  ldb_metadata_(nullptr),
  path_data_(relative_path + "/" + dir_name + LDB_DATA_POSTFIX),
  ldb_data_(nullptr),
  read_iter_(nullptr),
  topic_name_(topic_name),
  topic_type_(""),
  readwrite_(readwrite),
  remove_all_ldbs_(false),
  message_count_(0),
  cur_system_data_type_(message_data_type_e::LITTLE_ENDIAN_),
  message_data_type_(message_data_type_e::LITTLE_ENDIAN_),
  enable_endian_convert_(false)
{
  // Check endian type of current system.
  int check_endian = 1;
  if (*(reinterpret_cast<char *>(&check_endian)) != 1) {
    cur_system_data_type_ = message_data_type_e::BIG_ENDIAN_;
  }
}

LeveldbWrapper::~LeveldbWrapper()
{
  if (nullptr != read_iter_) {
    delete read_iter_;
  }

  if (nullptr != ldb_metadata_) {
    delete ldb_metadata_;
  }

  if (nullptr != ldb_data_) {
    delete ldb_data_;
  }

  // Check if leveldb need to be removed
  if (remove_all_ldbs_) {
    rcpputils::fs::remove_all(path_metadata_);
    rcpputils::fs::remove_all(path_data_);
  }
}

void inline LeveldbWrapper::prepare_read()
{
  if (nullptr == read_iter_) {
    read_iter_ = ldb_data_->NewIterator(leveldb::ReadOptions());
    read_iter_->SeekToFirst();
  }
}

void LeveldbWrapper::init_ldb()
{
  // Open metadata leveldb
  open_leveldb(path_metadata_, &ldb_metadata_, readwrite_);

  // If readonly mode
  if (!readwrite_) {
    // Check endian type of message leveldb
    std::string val;
    read_metadata(leveldb::Slice(KEY_ENDIAN), val);
    if (!val.compare(VALUE_BIG_ENDIAN)) {
      message_data_type_ = message_data_type_e::BIG_ENDIAN_;
    }

    // Get topic name
    read_metadata(leveldb::Slice(KEY_TOPIC_NAME), val);
    topic_name_ = val;

    if (message_data_type_ != cur_system_data_type_) {
      enable_endian_convert_ = true;
      ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_INFO_STREAM(
        "The endianness of message is different from current system."
        " Convert operation will be done.");
    }
  }

  // Open message leveldb
  open_leveldb(path_data_, &ldb_data_, readwrite_);

  if (!readwrite_) {
    prepare_read();
  }
}

void LeveldbWrapper::open_leveldb(const std::string & path, leveldb::DB ** ldb, bool readwrite)
{
  leveldb::Options options;
  if (readwrite) {
    options.create_if_missing = true;
  }

  // Disable compression function in leveldb
  options.compression = leveldb::kNoCompression;

  // For message ldb, timestamp comparator need to be used while open leveldb
  if (ldb == &ldb_data_) {
    // Only read message leveldb and need to do convert
    if (!readwrite && enable_endian_convert_) {
      timestamp_comparator_.enable_endian_convert();
    }
    options.comparator = &timestamp_comparator_;
  }

  if (!rcpputils::fs::path(path).exists()) {
    bool ret = rcpputils::fs::create_directories(path);
    if (!ret) {
      throw std::runtime_error("Failed to create directory " + path);
    }
  }

  leveldb::Status status = leveldb::DB::Open(options, path.c_str(), ldb);
  if (!status.ok()) {
    throw LeveldbException("Failed to open leveldb " + path + ": " + status.ToString());
  }
}

void LeveldbWrapper::msg_write(
  leveldb::DB * ldb,
  const leveldb::Slice & key,
  const leveldb::Slice & val)
{
  leveldb::Status status = ldb->Put(leveldb::WriteOptions(), key, val);
  if (!status.ok()) {
    throw LeveldbException(
            "Failed to write (" +
            key.ToString() + ":" + val.ToString() + ") leveldb :" + status.ToString());
  }
  message_count_++;
}

void LeveldbWrapper::msg_write(
  leveldb::DB * ldb,
  std::vector<std::pair<leveldb::Slice, leveldb::Slice>> & data)
{
  leveldb::WriteBatch batch;
  uint64_t count = 0;
  for (auto kv : data) {
    batch.Put(kv.first, kv.second);
    count++;
  }
  leveldb::Status status = ldb->Write(leveldb::WriteOptions(), &batch);
  if (!status.ok()) {
    throw LeveldbException("Failed to batch write :" + status.ToString());
  }
  message_count_ += count;
}

void LeveldbWrapper::write_metadata(const rosbag2_storage::TopicMetadata & topic_metadata)
{
  std::vector<std::pair<leveldb::Slice, leveldb::Slice>> metadata;

  metadata.emplace_back(
    std::pair<leveldb::Slice, leveldb::Slice>(
      KEY_TOPIC_NAME,
      topic_metadata.name));

  metadata.emplace_back(
    std::pair<leveldb::Slice, leveldb::Slice>(
      KEY_TOPIC_TYPE,
      topic_metadata.type));

  metadata.emplace_back(
    std::pair<leveldb::Slice, leveldb::Slice>(
      KEY_SERIALIZATION_FORMAT,
      topic_metadata.serialization_format));

  metadata.emplace_back(
    std::pair<leveldb::Slice, leveldb::Slice>(
      KEY_QOS_PROFILES,
      topic_metadata.offered_qos_profiles));

  if (message_data_type_ == message_data_type_e::LITTLE_ENDIAN_) {
    metadata.emplace_back(
      std::pair<leveldb::Slice, leveldb::Slice>(KEY_ENDIAN, VALUE_LITTLE_ENDIAN));
  } else {
    metadata.emplace_back(
      std::pair<leveldb::Slice, leveldb::Slice>(KEY_ENDIAN, VALUE_BIG_ENDIAN));
  }

  msg_write(ldb_metadata_, metadata);
}

inline void LeveldbWrapper::read_metadata(const leveldb::Slice & key, std::string & val)
{
  leveldb::Status status = ldb_metadata_->Get(leveldb::ReadOptions(), key, &val);
  if (!status.ok()) {
    throw LeveldbException(
            "Failed to read (key:" + key.ToString() + ") :" + status.ToString() +
            "In leveldb " + path_metadata_);
  }
}

std::shared_ptr<rosbag2_storage::TopicMetadata> LeveldbWrapper::read_all_metadata()
{
  std::shared_ptr<rosbag2_storage::TopicMetadata> topic_metadata =
    std::make_shared<rosbag2_storage::TopicMetadata>();

  read_metadata(leveldb::Slice(KEY_TOPIC_NAME), topic_metadata->name);
  read_metadata(leveldb::Slice(KEY_TOPIC_TYPE), topic_metadata->type);
  read_metadata(leveldb::Slice(KEY_SERIALIZATION_FORMAT), topic_metadata->serialization_format);
  read_metadata(leveldb::Slice(KEY_QOS_PROFILES), topic_metadata->offered_qos_profiles);

  return topic_metadata;
}

bool LeveldbWrapper::has_next()
{
  prepare_read();

  return read_iter_->Valid();
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> LeveldbWrapper::read_next()
{
  prepare_read();

  if (read_iter_->Valid()) {
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message;
    if (!read_iter_->value().empty() && !read_iter_->key().empty()) {
      bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

      // Read timestamp
      // If endian type of message database is different from current system
      if (enable_endian_convert_) {
        rcutils_time_point_value_t tmp;
        memcpy(&tmp, read_iter_->key().data(), sizeof(rcutils_time_point_value_t));
        bag_message->time_stamp = swap_64(tmp);
      } else {
        memcpy(
          &bag_message->time_stamp,
          read_iter_->key().data(),
          sizeof(rcutils_time_point_value_t));
      }

      // Read message
      bag_message->serialized_data =
        rosbag2_storage::make_serialized_message(
        read_iter_->value().data(),
        read_iter_->value().size());
    } else {
      throw LeveldbException("Read empty data from message leveldb !");
    }

    bag_message->topic_name = topic_name_;

    // Go to next key/value
    read_iter_->Next();

    return bag_message;
  }

  throw LeveldbException("No data from message leveldb !");
}

uint64_t LeveldbWrapper::get_topic_ldb_size()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  uint64_t metadata_ldb_size;
  if (RCUTILS_RET_OK !=
    rcutils_calculate_directory_size(path_metadata_.c_str(), &metadata_ldb_size, allocator))
  {
    throw LeveldbException("Failed to get the size of metadata leveldb !");
  }

  uint64_t message_ldb_size;
  if (RCUTILS_RET_OK !=
    rcutils_calculate_directory_size(path_data_.c_str(), &message_ldb_size, allocator))
  {
    throw LeveldbException("Failed to get the size of metadata leveldb !");
  }

  return metadata_ldb_size + message_ldb_size;
}

void LeveldbWrapper::remove_database()
{
  remove_all_ldbs_ = true;
}

void LeveldbWrapper::write_message(
  const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> & message)
{
  msg_write(
    ldb_data_,
    leveldb::Slice(
      reinterpret_cast<const char *>(&message->time_stamp),
      sizeof(rcutils_time_point_value_t)),
    leveldb::Slice(
      reinterpret_cast<const char *>(message->serialized_data->buffer),
      message->serialized_data->buffer_length));
}

void LeveldbWrapper::write_message(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & messages)
{
  std::vector<std::pair<leveldb::Slice, leveldb::Slice>> msg_slices;
  for (auto msg : messages) {
    msg_slices.emplace_back(
      leveldb::Slice(
        reinterpret_cast<const char *>(&msg->time_stamp),
        sizeof(rcutils_time_point_value_t)),
      leveldb::Slice(
        reinterpret_cast<char *>(msg->serialized_data->buffer),
        msg->serialized_data->buffer_length));
  }
  msg_write(ldb_data_, msg_slices);
}

size_t LeveldbWrapper::get_message_count() const
{
  if (readwrite_) {
    // readwrite mode
    return static_cast<size_t>(message_count_);
  } else {
    // readonly mode, need to get from message leveldb
    if (ldb_data_ != nullptr) {
      uint64_t count = 0;
      std::shared_ptr<leveldb::Iterator> iter(ldb_data_->NewIterator(leveldb::ReadOptions()));
      for (iter->SeekToFirst(); iter->Valid(); iter->Next()) {
        count++;
      }
      return count;
    } else {
      throw LeveldbException("Cannot get message since message leveldb isn't opened !");
    }
  }
}

rcutils_time_point_value_t LeveldbWrapper::get_min_timestamp()
{
  std::shared_ptr<leveldb::Iterator> iter(ldb_data_->NewIterator(leveldb::ReadOptions()));
  iter->SeekToFirst();
  return get_timestamp(iter);
}

rcutils_time_point_value_t LeveldbWrapper::get_max_timestamp()
{
  std::shared_ptr<leveldb::Iterator> iter(ldb_data_->NewIterator(leveldb::ReadOptions()));
  iter->SeekToLast();
  return get_timestamp(iter);
}

rcutils_time_point_value_t LeveldbWrapper::get_timestamp(std::shared_ptr<leveldb::Iterator> & iter)
{
  rcutils_time_point_value_t tmp;
  if (iter->Valid()) {
    memcpy(&tmp, iter->value().data(), sizeof(rcutils_time_point_value_t));
    if (enable_endian_convert_) {
      return tmp;
    } else {
      return swap_64(tmp);
    }
  }

  return 0;
}

std::string LeveldbWrapper::get_topic_name() const
{
  return topic_name_;
}

}  // namespace rosbag2_storage_plugins
