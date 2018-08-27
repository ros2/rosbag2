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

#ifndef ROSBAG2__ROSBAG2_INTEGRATION_TEST_FIXTURE_HPP_
#define ROSBAG2__ROSBAG2_INTEGRATION_TEST_FIXTURE_HPP_

#include <gmock/gmock.h>

#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/rosbag2.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

#include "rosbag2_test_fixture.hpp"
#include "test_memory_management.hpp"

using namespace std::chrono_literals;  // NOLINT

class RosBag2IntegrationTestFixture : public Rosbag2TestFixture
{
public:
  RosBag2IntegrationTestFixture()
  : Rosbag2TestFixture(), memory_(), counter_(0)
  {}

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void record_message(const std::string & db_name, std::vector<std::string> topics)
  {
    rosbag2::Rosbag2 rosbag2;
    rosbag2.record(db_name, topics, [this]() {
        counter_++;
      });
  }

  void start_recording(std::vector<std::string> topics)
  {
    // the future object returned from std::async needs to be stored not to block the execution
    future_ = std::async(
      std::launch::async, [this, topics]() {
        record_message(database_name_, topics);
      });
  }

  void stop_recording()
  {
    publisher_future_.get();
    rclcpp::shutdown();
    future_.get();
  }

  void start_publishing(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> message,
    std::string topic_name)
  {
    publisher_future_ = std::async(
      std::launch::async, [this, message, topic_name]() {
        auto node = std::make_shared<rclcpp::Node>("publisher_node");
        auto publisher = node->create_publisher<std_msgs::msg::String>(topic_name);
        auto timer = node->create_wall_timer(50ms, [publisher, message]() {
          publisher->publish(message->serialized_data.get());
        });

        while (counter_ < 1) {
          rclcpp::spin_some(node);
        }
      }
    );
  }

  template<typename T>
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialize_message(
    std::shared_ptr<T> message, std::string topic_name)
  {
    auto serialized_message = memory_.serialize_message<T>(message);

    auto serialized_bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    serialized_bag_message->serialized_data = serialized_message;
    serialized_bag_message->topic_name = topic_name;
    return serialized_bag_message;
  }

  template<typename T>
  std::shared_ptr<T> deserialize_message(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
  {
    return memory_.deserialize_message<T>(message->serialized_data);
  }

  test_helpers::TestMemoryManagement memory_;
  std::atomic<size_t> counter_;
  std::future<void> publisher_future_;
  std::future<void> future_;
};

#endif  // ROSBAG2__ROSBAG2_INTEGRATION_TEST_FIXTURE_HPP_
