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

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"

#include "rosbag2_test_common/memory_management.hpp"

#include "rosbag2_transport_test_fixture.hpp"

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
    stop_spinning();
    rclcpp::shutdown();
  }

  template<class T>
  void start_async_spin(T node)
  {
    if (exec_ == nullptr) {
      exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      exec_->add_node(node);
      spin_thread_ = std::thread(
        [this]() {
          exec_->spin();
        });
      // Wait for the executor to start spinning in the newly spawned thread to avoid race condition
      // with exec_->cancel()
      using clock = std::chrono::steady_clock;
      auto start = clock::now();
      while (!exec_->is_spinning() && (clock::now() - start) < std::chrono::seconds(5)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (!exec_->is_spinning()) {
        throw std::runtime_error("Failed to start spinning node");
      }
    } else {
      throw std::runtime_error("Already spinning a node, can't start a new node spin");
    }
  }

  void stop_spinning()
  {
    if (exec_ != nullptr) {
      exec_->cancel();
      if (spin_thread_.joinable()) {
        spin_thread_.join();
      }
      exec_ = nullptr;
    }
  }

  template<typename MessageT>
  std::vector<std::shared_ptr<MessageT>> filter_messages(
    std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> messages,
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

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_{nullptr};
  MemoryManagement memory_;
  std::thread spin_thread_;
};

#endif  // ROSBAG2_TRANSPORT__RECORD_INTEGRATION_FIXTURE_HPP_
