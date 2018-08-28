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

#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/rosbag2.hpp"
#include "std_msgs/msg/string.hpp"

#include "rosbag2_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

// TODO(Martin-Idel-SI): merge w. rosbag2_write_integration_test once signal handling is sorted out
class RosBag2IntegrationTestFixture : public Rosbag2TestFixture
{
public:
  RosBag2IntegrationTestFixture()
  : Rosbag2TestFixture(), messages_stored_counter_(0)
  {
    rclcpp::init(0, nullptr);
  }

  ~RosBag2IntegrationTestFixture() override
  {
    rclcpp::shutdown();
  }

  template<typename T>
  auto subscribe_messages(
    size_t expected_messages_number, std::string topic)
  {
    auto node = rclcpp::Node::make_shared("node_" + topic);
    std::vector<typename T::_data_type> messages;
    size_t messages_received = 0;
    auto subscription = node->create_subscription<T>(topic,
        [&messages, &messages_received](typename T::ConstSharedPtr message) {
          messages.push_back(message->data);
          ++messages_received;
          std::cout << "received message: " << message->data << std::endl;
        });
    subscriptions_.push_back(subscription);

    while (messages_received < expected_messages_number) {
      rclcpp::spin_some(node);
    }
    return messages;
  }

  template<typename MessageT>
  auto launch_subscriber(size_t expected_messages_number, std::string topic)
  {
    return std::async(std::launch::async, [this, expected_messages_number, topic] {
               return subscribe_messages<MessageT>(expected_messages_number, topic);
             });
  }

  void wait_for_subscribers(size_t count)
  {
    std::async(std::launch::async, [this, count] {
        while (subscriptions_.size() < count) {
          std::this_thread::sleep_for(50ms);
        }
      }).get();
  }

  void play_bag(std::string database_name)
  {
    rosbag2::Rosbag2 rosbag2;
    rosbag2.play(database_name);
  }

  std::atomic<size_t> messages_stored_counter_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

TEST_F(RosBag2IntegrationTestFixture, recorded_messages_are_played_for_all_topics)
{
  auto topic_types = std::map<std::string, std::string>{
    {"topic1", "std_msgs/String"},
    {"topic2", "std_msgs/UInt8"},
  };
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_message<std_msgs::msg::String>("topic1", "Hello World 1"),
    serialize_message<std_msgs::msg::String>("topic1", "Hello World 2"),
    serialize_message<std_msgs::msg::String>("topic1", "Hello World 2"),
    serialize_message<std_msgs::msg::UInt8>("topic2", 123),
    serialize_message<std_msgs::msg::UInt8>("topic2", 127),
    serialize_message<std_msgs::msg::UInt8>("topic2", 127)};

  write_messages(database_name_, messages, topic_types);

  // Due to a problem related to the subscriber, we play many (3) messages but make the subscriber
  // node spin only until 2 have arrived. Hence the 2 as `launch_subscriber()` argument.
  auto string_subscriber_future = launch_subscriber<std_msgs::msg::String>(2, "topic1");
  auto int_subscriber_future = launch_subscriber<std_msgs::msg::UInt8>(2, "topic2");
  wait_for_subscribers(2);
  play_bag(database_name_);

  auto replayed_string_messages = string_subscriber_future.get();
  ASSERT_THAT(replayed_string_messages, SizeIs(2));
  ASSERT_THAT(replayed_string_messages[0], Eq("Hello World 1"));
  ASSERT_THAT(replayed_string_messages[1], Eq("Hello World 2"));
  auto replayed_int_messages = int_subscriber_future.get();
  ASSERT_THAT(replayed_int_messages, SizeIs(2));
  ASSERT_THAT(replayed_int_messages[0], Eq(123));
  ASSERT_THAT(replayed_int_messages[1], Eq(127));
}
