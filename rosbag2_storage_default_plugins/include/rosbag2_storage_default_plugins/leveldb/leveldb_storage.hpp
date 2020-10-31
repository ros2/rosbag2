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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__LEVELDB__LEVELDB_STORAGE_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__LEVELDB__LEVELDB_STORAGE_HPP_

#include <leveldb/db.h>
#include <leveldb/status.h>

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <map>
#include <cassert>

#include "rcutils/types.h"

#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag2_storage_default_plugins/visibility_control.hpp"
#include "rosbag2_storage_default_plugins/leveldb/leveldb_wrapper.hpp"

namespace rosbag2_storage_plugins
{

class ROSBAG2_STORAGE_DEFAULT_PLUGINS_PUBLIC LeveldbStorage
  : public rosbag2_storage::storage_interfaces::ReadWriteInterface
{
public:
  LeveldbStorage() = default;

  ~LeveldbStorage() override;

  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    rosbag2_storage::storage_interfaces::IOFlag io_flag =
    rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;

  void remove_topic(const rosbag2_storage::TopicMetadata & topic) override;

  void create_topic(const rosbag2_storage::TopicMetadata & topic) override;

  void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message) override;

  void write(
    const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & messages)
  override;

  bool has_next() override;

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;

  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() override;

  rosbag2_storage::BagMetadata get_metadata() override;

  std::string get_relative_file_path() const override;

  uint64_t get_bagfile_size() const override;

  std::string get_storage_identifier() const override;

  uint64_t get_minimum_split_file_size() const override;

  void set_filter(const rosbag2_storage::StorageFilter & storage_filter) override;

  void reset_filter() override;

private:
  std::map<std::string, std::shared_ptr<class LeveldbWrapper>> topic_ldb_map_{};
  std::vector<rosbag2_storage::TopicMetadata> all_topics_and_types_{};
  std::string relative_path_;
  rosbag2_storage::StorageFilter storage_filter_{};
  bool scan_ldb_done_{false};

  struct ts_compare
  {
    bool operator()(
      const rcutils_time_point_value_t & ts1,
      const rcutils_time_point_value_t & ts2)
    {
      return ts1 < ts2;
    }
  };
  std::map<
    rcutils_time_point_value_t,
    std::shared_ptr<rosbag2_storage::SerializedBagMessage>,
    ts_compare> read_cache_{};

  inline void scan_ldb_for_read();
  void fill_topics_and_types();
};

}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__LEVELDB__LEVELDB_STORAGE_HPP_
