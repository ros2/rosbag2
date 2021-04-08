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

#ifndef ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_TEST_FIXTURE_HPP_
#define ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_TEST_FIXTURE_HPP_

#include <gtest/gtest.h>

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/types.hpp"
#include "rosbag2_cpp/writer.hpp"

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#endif

#include "rosbag2_storage/storage_options.hpp"

#include "rosbag2_transport/play_options.hpp"

#include "rosbag2_test_common/memory_management.hpp"

#include "mock_sequential_reader.hpp"
#include "mock_sequential_writer.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common; // NOLINT

class Rosbag2TransportTestFixture : public Test
{
public:
  Rosbag2TransportTestFixture()
  : storage_options_({"uri", "storage_id", 0, 100}), play_options_({1000}),
    reader_(std::make_shared<rosbag2_cpp::Reader>(std::make_unique<MockSequentialReader>())),
    writer_(std::make_shared<rosbag2_cpp::Writer>(std::make_unique<MockSequentialWriter>())) {}

  template<typename MessageT>
  std::shared_ptr<rosbag2_storage::SerializedBagMessage>
  serialize_test_message(
    const std::string & topic,
    int64_t milliseconds,
    std::shared_ptr<MessageT> message)
  {
    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_msg->serialized_data = memory_management_.serialize_message(message);
    bag_msg->time_stamp = milliseconds * 1000000;
    bag_msg->topic_name = topic;

    return bag_msg;
  }

  MemoryManagement memory_management_;

  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::PlayOptions play_options_;

  std::shared_ptr<rosbag2_cpp::Reader> reader_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
};

#endif  // ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_TEST_FIXTURE_HPP_
