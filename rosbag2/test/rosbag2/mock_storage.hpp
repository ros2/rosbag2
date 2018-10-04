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

#ifndef ROSBAG2__MOCK_STORAGE_HPP_
#define ROSBAG2__MOCK_STORAGE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/topic_with_type.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

class MockStorage : public rosbag2_storage::storage_interfaces::ReadWriteInterface
{
public:
  ~MockStorage() override = default;

  void open(const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag flag) override
  {
    (void) uri;
    (void) flag;
  }

  void create_topic(const rosbag2_storage::TopicWithType & topic) override
  {
    (void) topic;
  }

  bool has_next() override {return true;}

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override {return nullptr;}

  void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) override
  {
    (void) msg;
  }

  std::vector<rosbag2_storage::TopicWithType> get_all_topics_and_types() override
  {
    return std::vector<rosbag2_storage::TopicWithType>();
  }

  rosbag2_storage::BagMetadata get_metadata() override
  {
    return rosbag2_storage::BagMetadata();
  }
};

#endif  // ROSBAG2__MOCK_STORAGE_HPP_
