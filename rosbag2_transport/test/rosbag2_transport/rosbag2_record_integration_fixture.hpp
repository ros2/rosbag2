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
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_transport/rosbag2_transport.hpp"
#include "rosbag2/types.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_exception.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.hpp"

#include "rosbag2_test_fixture.hpp"
#include "test_memory_management.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

#ifndef ROSBAG2_TRANSPORT__ROSBAG2_RECORD_INTEGRATION_FIXTURE_HPP_
#define ROSBAG2_TRANSPORT__ROSBAG2_RECORD_INTEGRATION_FIXTURE_HPP_

class RosBag2RecordIntegrationTestFixture : public Rosbag2TestFixture
{
public:
  RosBag2RecordIntegrationTestFixture()
  : Rosbag2TestFixture()
  {
    rclcpp::init(0, nullptr);
    publisher_node_ = rclcpp::Node::make_shared("publisher_node");
  }

  void start_recording(std::vector<std::string> topics)
  {
    // the future object returned from std::async needs to be stored not to block the execution
    future_ = std::async(
      std::launch::async, [this, topics]() {
        rosbag2_transport::Rosbag2Transport rosbag2_transport(reader_, writer_);
        rosbag2_transport.record(storage_options_, {false, topics});
      });
  }

  void start_recording_all_topics()
  {
    // the future object returned from std::async needs to be stored not to block the execution
    future_ = std::async(
      std::launch::async, [this]() {
        rosbag2_transport::Rosbag2Transport rosbag2_transport(reader_, writer_);
        rosbag2_transport.record(storage_options_, {true, {}});
      });
  }

  void stop_recording()
  {
    rclcpp::shutdown();
    future_.get();
  }

  template<class T>
  auto create_publisher(
    const std::string & topic_name, std::shared_ptr<T> message, size_t expected_messages)
  {
    static int counter = 0;
    auto publisher_node = std::make_shared<rclcpp::Node>("publisher" + std::to_string(counter++));
    auto publisher = publisher_node->create_publisher<T>(topic_name);

    // We need to publish one message to set up the topic for discovery
    publisher->publish(message);

    return [this, publisher, topic_name, message, expected_messages]() {
             while (rclcpp::ok() && writer_->messages_per_topic()[topic_name] < expected_messages) {
               publisher->publish(message);
               // rate limiting
               std::this_thread::sleep_for(50ms);
             }
           };
  }

  void run_publishers(std::initializer_list<std::function<void()>> publishers)
  {
    std::vector<std::future<void>> futures;
    for (const auto & publisher : publishers) {
      futures.push_back(std::async(std::launch::async, publisher));
    }
    for (auto & publisher_future : futures) {
      publisher_future.get();
    }
  }

  template<typename MessageT>
  std::shared_ptr<MessageT> deserialize_message(
    std::shared_ptr<rosbag2::SerializedBagMessage> message)
  {
    return memory_.deserialize_message<MessageT>(message->serialized_data);
  }

  template<typename MessageT>
  std::vector<std::shared_ptr<MessageT>> filter_messages(
    std::vector<std::shared_ptr<rosbag2::SerializedBagMessage>> messages,
    const std::string & topic)
  {
    std::vector<std::shared_ptr<MessageT>> filtered_messages;
    for (const auto & message : messages) {
      if (message->topic_name == topic) {
        filtered_messages.push_back(deserialize_message<MessageT>(message));
      }
    }
    return filtered_messages;
  }

  rclcpp::Node::SharedPtr publisher_node_;
  test_helpers::TestMemoryManagement memory_;
  std::vector<std::future<void>> publisher_futures_;
  std::future<void> future_;
};

#endif  // ROSBAG2_TRANSPORT__ROSBAG2_RECORD_INTEGRATION_FIXTURE_HPP_
