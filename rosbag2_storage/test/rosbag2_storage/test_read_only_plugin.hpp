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

#ifndef ROSBAG2_STORAGE__TEST_READ_ONLY_PLUGIN_HPP_
#define ROSBAG2_STORAGE__TEST_READ_ONLY_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"

class TestReadOnlyPlugin : public rosbag2_storage::storage_interfaces::ReadOnlyInterface
{
public:
  ~TestReadOnlyPlugin() override;

  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    rosbag2_storage::storage_interfaces::IOFlag flag) override;

  bool set_read_order(const rosbag2_storage::ReadOrder &) override;

  bool has_next() override;

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;

  std::vector<std::pair<rosbag2_storage::TopicMetadata,
    rosbag2_storage::MessageDefinition>> get_all_topics_and_types() override;

  rosbag2_storage::BagMetadata get_metadata() override;

  std::string get_relative_file_path() const override;

  uint64_t get_bagfile_size() const override;

  std::string get_storage_identifier() const override;

  void set_filter(const rosbag2_storage::StorageFilter & storage_filter) override;

  void reset_filter() override;

  void seek(const rcutils_time_point_value_t & timestamp) override;
};

#endif  // ROSBAG2_STORAGE__TEST_READ_ONLY_PLUGIN_HPP_
