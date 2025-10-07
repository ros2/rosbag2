// Copyright 2024 Virginia Tech Transportation Institute - Jean Paul Talledo Vilela
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

#ifndef ROSBAG2_STORAGE_FIFO_PLUGIN__FIFO_STORAGE_HPP_
#define ROSBAG2_STORAGE_FIFO_PLUGIN__FIFO_STORAGE_HPP_

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

// This is the correct header for Humble to look up message definitions
#include "rosbag2_storage_mcap/message_definition_cache.hpp"

#include "mcap/writer.hpp"
#include "yaml-cpp/yaml.h"

namespace rosbag2_storage_fifo_plugin
{
class FdWriter : public mcap::IWritable
{
public:
  explicit FdWriter(int fd)
  : fd_(fd), total_bytes_written_(0) {}

  ~FdWriter() override = default;

  void handleWrite(const std::byte * data, uint64_t size) override;
  uint64_t size() const override;
  void end() override {}

private:
  int fd_;
  uint64_t total_bytes_written_;
};

class FifoStorage : public rosbag2_storage::storage_interfaces::ReadWriteInterface
{
public:
  FifoStorage();
  ~FifoStorage() override;

  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    rosbag2_storage::storage_interfaces::IOFlag io_flag =
    rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;

  void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message) override;
  void write(
    const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & messages)
  override;

  void create_topic(const rosbag2_storage::TopicMetadata & topic) override;
  void remove_topic(const rosbag2_storage::TopicMetadata & topic) override;
  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() override;
  rosbag2_storage::BagMetadata get_metadata() override;

  bool has_next() override {return false;}
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override {return nullptr;}
  void seek(const rcutils_time_point_value_t & timestamp) override {(void) timestamp;}

  uint64_t get_bagfile_size() const override;
  std::string get_storage_identifier() const override;
  std::string get_relative_file_path() const override;

  void set_filter(const rosbag2_storage::StorageFilter & storage_filter) override;
  void reset_filter() override;
  uint64_t get_minimum_split_file_size() const override;

private:
  void create_fifo(const std::string & path);
  void write_metadata_to_yaml_fifo();

  std::string mcap_fifo_path_;
  std::string yaml_fifo_path_;
  int mcap_fifo_fd_ = -1;

  std::unique_ptr<FdWriter> mcap_fd_writer_;
  std::unique_ptr<mcap::McapWriter> mcap_writer_;
  std::unique_ptr<rosbag2_storage_mcap::internal::MessageDefinitionCache> definition_cache_;

  rosbag2_storage::BagMetadata metadata_{};
  std::vector<rosbag2_storage::TopicMetadata> topics_;
  std::unordered_map<std::string, mcap::ChannelId> topic_to_channel_id_;
  std::unordered_map<std::string, uint64_t> message_counts_;

  rcutils_time_point_value_t starting_time_ = 0;
  rcutils_time_point_value_t last_timestamp_ = 0;
  const std::string storage_identifier_ = "fifo";
  rclcpp::Logger logger_;
};
}  // namespace rosbag2_storage_fifo_plugin

#endif  // ROSBAG2_STORAGE_FIFO_PLUGIN__FIFO_STORAGE_HPP_
