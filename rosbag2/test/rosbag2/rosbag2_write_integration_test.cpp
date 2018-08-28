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

#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/rosbag2.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "rosbag2_test_fixture.hpp"
#include "test_memory_management.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class RosBag2IntegrationTestFixture : public Rosbag2TestFixture
{
public:
  RosBag2IntegrationTestFixture()
  : Rosbag2TestFixture(), memory_(), messages_stored_counter_(0)
  {}

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void record_message(const std::string & db_name, std::vector<std::string> topics)
  {
    rosbag2::Rosbag2 rosbag2;
    rosbag2.record(db_name, topics);
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
    for (auto & future : publisher_futures_) {
      future.get();
    }
    rclcpp::shutdown();
    future_.get();
  }

  void start_publishing(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> message,
    std::string topic_name)
  {
    // We just publish three messages. This should be enough for all practical purposes
    std::weak_ptr<rosbag2_storage::SerializedBagMessage> message_weak_ptr(message);
    publisher_futures_.push_back(std::async(
        std::launch::async, [message_weak_ptr, topic_name]() {
          auto message_lock = message_weak_ptr.lock();
          auto node = std::make_shared<rclcpp::Node>("publisher_node");
          auto publisher = node->create_publisher<std_msgs::msg::String>(topic_name);
          publisher->publish(message_lock->serialized_data.get());
          std::this_thread::sleep_for(150ms);
          publisher->publish(message_lock->serialized_data.get());
          std::this_thread::sleep_for(150ms);
          publisher->publish(message_lock->serialized_data.get());
        }
    ));
  }

  template<typename MessageT>
  std::shared_ptr<MessageT> deserialize_message(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
  {
    return memory_.deserialize_message<MessageT>(message->serialized_data);
  }

  test_helpers::TestMemoryManagement memory_;
  std::atomic<size_t> messages_stored_counter_;
  std::vector<std::future<void>> publisher_futures_;
  std::future<void> future_;
};

// TODO(Martin-Idel-SI): merge with other write and read tests once signal handling is sorted out
TEST_F(RosBag2IntegrationTestFixture, published_messages_from_multiple_topics_are_recorded)
{
  std::string int_topic = "/int_topic";
  auto serialized_int_bag_message = serialize_message<std_msgs::msg::UInt8>(int_topic, 10);

  std::string string_topic = "/string_topic";
  auto serialized_string_bag_message = serialize_message<std_msgs::msg::String>(
    string_topic, "test_message");

  start_publishing(serialized_string_bag_message, string_topic);
  start_publishing(serialized_int_bag_message, int_topic);
  start_recording({string_topic, int_topic});
  stop_recording();

  auto recorded_messages = get_messages(database_name_);

  ASSERT_THAT(recorded_messages, Not(IsEmpty()));
  std::vector<std::shared_ptr<std_msgs::msg::String>> string_messages;
  std::vector<std::shared_ptr<std_msgs::msg::UInt8>> int_messages;
  for (const auto & message : recorded_messages) {
    if (message->topic_name == string_topic) {
      auto deserialized = deserialize_message<std_msgs::msg::String>(message);
      string_messages.push_back(deserialized);
    } else if (message->topic_name == int_topic) {
      auto deserialized = deserialize_message<std_msgs::msg::UInt8>(message);
      int_messages.push_back(deserialized);
    }
  }
  ASSERT_THAT(string_messages, Not(IsEmpty()));
  ASSERT_THAT(int_messages, Not(IsEmpty()));
  EXPECT_THAT(string_messages[0]->data, Eq("test_message"));
  EXPECT_THAT(int_messages[0]->data, Eq(10));
}
