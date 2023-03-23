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

#ifndef ROSBAG2_CPP__MOCK_STORAGE_HPP_
#define ROSBAG2_CPP__MOCK_STORAGE_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

class MockStorage : public rosbag2_storage::storage_interfaces::ReadWriteInterface
{
public:
  MOCK_METHOD2(
    open,
    void(const rosbag2_storage::StorageOptions &, rosbag2_storage::storage_interfaces::IOFlag));
  MOCK_METHOD1(update_metadata, void(const rosbag2_storage::BagMetadata &));
  MOCK_METHOD1(register_message_definition, void(const rosbag2_storage::MessageDefinition &));
  MOCK_METHOD2(
    create_topic, void(const rosbag2_storage::TopicMetadata &,
    const rosbag2_storage::MessageDefinition &));
  MOCK_METHOD1(remove_topic, void(const rosbag2_storage::TopicMetadata &));
  MOCK_METHOD1(set_read_order, bool(const rosbag2_storage::ReadOrder &));
  MOCK_METHOD0(has_next, bool());
  MOCK_METHOD0(has_next_file, bool());
  MOCK_METHOD0(read_next, std::shared_ptr<rosbag2_storage::SerializedBagMessage>());
  MOCK_METHOD1(write, void(std::shared_ptr<const rosbag2_storage::SerializedBagMessage>));
  MOCK_METHOD1(
    write,
    void(const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> &));
  MOCK_METHOD0(get_all_topics_and_types, std::vector<rosbag2_storage::TopicMetadata>());
  MOCK_METHOD0(get_metadata, rosbag2_storage::BagMetadata());
  MOCK_METHOD0(reset_filter, void());
  MOCK_METHOD1(set_filter, void(const rosbag2_storage::StorageFilter &));
  MOCK_METHOD1(seek, void(const rcutils_time_point_value_t &));
  MOCK_CONST_METHOD0(get_bagfile_size, uint64_t());
  MOCK_CONST_METHOD0(get_relative_file_path, std::string());
  MOCK_CONST_METHOD0(get_storage_identifier, std::string());
  MOCK_CONST_METHOD0(get_storage_extension, std::string());
  MOCK_CONST_METHOD0(get_minimum_split_file_size, uint64_t());
};

#endif  // ROSBAG2_CPP__MOCK_STORAGE_HPP_
