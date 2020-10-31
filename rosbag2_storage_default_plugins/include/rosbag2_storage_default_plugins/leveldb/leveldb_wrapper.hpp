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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__LEVELDB__LEVELDB_WRAPPER_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__LEVELDB__LEVELDB_WRAPPER_HPP_

#include <leveldb/db.h>
#include <leveldb/write_batch.h>
#include <leveldb/comparator.h>

#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <memory>

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage_default_plugins/leveldb/leveldb_exception.hpp"
#include "rosbag2_storage_default_plugins/visibility_control.hpp"

namespace rosbag2_storage_plugins
{

#define swap_64(x) \
  ((((x) & 0xff00000000000000ull) >> 56) \
  | (((x) & 0x00ff000000000000ull) >> 40) \
  | (((x) & 0x0000ff0000000000ull) >> 24) \
  | (((x) & 0x000000ff00000000ull) >> 8) \
  | (((x) & 0x00000000ff000000ull) << 8) \
  | (((x) & 0x0000000000ff0000ull) << 24) \
  | (((x) & 0x000000000000ff00ull) << 40) \
  | (((x) & 0x00000000000000ffull) << 56))

class ROSBAG2_STORAGE_DEFAULT_PLUGINS_PUBLIC LeveldbWrapper
{
public:
  LeveldbWrapper(
    const std::string relative_path_,
    const std::string topic_name,
    const std::string dir_name,
    bool readwrite);

  ~LeveldbWrapper();

  void init_ldb();

  void write_metadata(const rosbag2_storage::TopicMetadata & topic_metadata);

  std::shared_ptr<rosbag2_storage::TopicMetadata> read_all_metadata();

  void write_message(const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> & message);

  void write_message(
    const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & messages);

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next();

  bool has_next();

  std::string get_topic_name() const;

  uint64_t get_topic_ldb_size();

  void remove_database();

  size_t get_message_count() const;

  rcutils_time_point_value_t get_min_timestamp();
  rcutils_time_point_value_t get_max_timestamp();

private:
  std::string path_metadata_;
  leveldb::DB * ldb_metadata_;
  std::string path_data_;
  leveldb::DB * ldb_data_;
  leveldb::Iterator * read_iter_;  // For message leveldb. Used for read mode.
  std::string topic_name_;
  std::string topic_type_;
  bool readwrite_;  // Open mode
  bool remove_all_ldbs_;
  uint64_t message_count_;  // Used for readwrite mode

  enum class message_data_type_e
  {
    BIG_ENDIAN_,
    LITTLE_ENDIAN_
  };
  message_data_type_e cur_system_data_type_;
  message_data_type_e message_data_type_;
  bool enable_endian_convert_;

  // This class is used for ordering timestamp(key) for message ldb
  class TSComparator : public leveldb::Comparator
  {
public:
    int Compare(const leveldb::Slice & a, const leveldb::Slice & b) const
    {
      rcutils_time_point_value_t ts1, ts2;
      ParseKey(a, ts1);
      ParseKey(b, ts2);
      if (ts1 < ts2) {
        return -1;
      } else if (ts1 > ts2) {
        return 1;
      } else {
        return 0;
      }
    }
    void enable_endian_convert()
    {
      enable_endian_convert_ = true;
    }
    // Ignore the following methods for now:
    const char * Name() const {return "Timestamp Comparator";}
    void FindShortestSeparator(std::string *, const leveldb::Slice &) const {}
    void FindShortSuccessor(std::string *) const {}

private:
    bool enable_endian_convert_{false};
    void ParseKey(const leveldb::Slice & k, rcutils_time_point_value_t & ts) const
    {
      memcpy(&ts, k.data(), sizeof(rcutils_time_point_value_t));
      if (enable_endian_convert_) {
        ts = swap_64(ts);
      }
    }
  };
  TSComparator timestamp_comparator_;

  void open_leveldb(const std::string & path, leveldb::DB ** ldb, bool readwrite);
  void msg_write(leveldb::DB * ldb, const leveldb::Slice & key, const leveldb::Slice & val);
  void msg_write(leveldb::DB * ldb, std::vector<std::pair<leveldb::Slice, leveldb::Slice>> & data);
  inline void read_metadata(const leveldb::Slice & key, std::string & val);
  void read_message(leveldb::Slice & key, leveldb::Slice & val);
  rcutils_time_point_value_t get_timestamp(std::shared_ptr<leveldb::Iterator> & iter);
  inline void prepare_read();
};

// The direcotry {TOPIC_NAME}_metadata for metadata leveldb
#define LDB_METADATA_POSTFIX "_ldb_metadata"
// The directory {TOPIC_NAME}_data for message leveldb
#define LDB_DATA_POSTFIX "_ldb_message"

// In metadata leveldb, include below keys
// KEY:
// N -- topic name
// T -- topic type
// SF  -- serialization format
// QP -- offered QOS profiles
// E -- B:big endian or L:little endian
#define KEY_TOPIC_NAME            "N"
#define KEY_TOPIC_TYPE            "T"
#define KEY_SERIALIZATION_FORMAT  "SF"
#define KEY_QOS_PROFILES          "QF"
#define KEY_ENDIAN                "E"
#define VALUE_LITTLE_ENDIAN       "L"
#define VALUE_BIG_ENDIAN          "B"
}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__LEVELDB__LEVELDB_WRAPPER_HPP_
