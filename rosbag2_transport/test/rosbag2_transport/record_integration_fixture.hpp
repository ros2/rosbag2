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

#include <gmock/gmock.h>

#include <future>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_transport/record_options.hpp"

#include "rosbag2_test_common/memory_management.hpp"

#include "rosbag2_transport_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

#ifndef ROSBAG2_TRANSPORT__RECORD_INTEGRATION_FIXTURE_HPP_
#define ROSBAG2_TRANSPORT__RECORD_INTEGRATION_FIXTURE_HPP_

class RecordIntegrationTestFixture : public Rosbag2TransportTestFixture
{
public:
  RecordIntegrationTestFixture()
  : Rosbag2TransportTestFixture()
  {
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  template<class T>
  void start_async_spin(T node)
  {
    future_ = std::async(
      std::launch::async, [node]() -> void {rclcpp::spin(node);});
  }

  void stop_spinning()
  {
    rclcpp::shutdown();
    if (future_.valid()) {
      future_.wait();
    }
  }

  template<typename MessageT>
  std::vector<std::shared_ptr<MessageT>> filter_messages(
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages,
    const std::string & topic)
  {
    std::vector<std::shared_ptr<MessageT>> filtered_messages;
    for (const auto & message : messages) {
      if (message->topic_name == topic) {
        filtered_messages.push_back(
          memory_.deserialize_message<MessageT>(message->serialized_data));
      }
    }
    return filtered_messages;
  }

  MemoryManagement memory_;
  std::future<void> future_;
};

#endif  // ROSBAG2_TRANSPORT__RECORD_INTEGRATION_FIXTURE_HPP_
