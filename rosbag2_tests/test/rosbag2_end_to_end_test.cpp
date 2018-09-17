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

#include <iostream>
#include <memory>
#include <future>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/primitives.hpp"
#include "test_msgs/msg/static_array_primitives.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace ::testing;  // NOLINT

class EndToEndTestFixture : public Test
{
public:
  EndToEndTestFixture()
  {
    remove("test.bag");
    rclcpp::init(0, nullptr);
  }

  ~EndToEndTestFixture() override
  {
    rclcpp::shutdown();
  }

  void record_all_topics()
  {
    auto process_id = fork();
    if (process_id == 0) {
      setpgid(getpid(), getpid());
      system("ros2 bag record -a");
    } else {
      // Here we wait to allow rosbag2 to record some messages before killing the process.
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      kill(-process_id, SIGINT);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      kill(-process_id, SIGTERM);
    }
  }

  void play_bag()
  {
    system("ros2 bag play test.bag");
  }

  std::future<void> publish_test_message()
  {
    return async(std::launch::async, []() {
      auto publisher_node = std::make_shared<rclcpp::Node>("publisher_node");
      auto publisher = publisher_node->create_publisher<test_msgs::msg::Primitives>("test_topic");
      auto message = get_messages_primitives()[0];
      message->string_value = "test";
      for (int i = 0; i < 10; i++) {
        publisher->publish(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    });
  }

  std::future<std::vector<std::string>> subscribe_and_wait_for_one_test_message()
  {
    return async(
      std::launch::async, []() -> std::vector<std::string> {
        std::vector<std::string> messages;
        int counter = 0;
        auto subscriber_node = std::make_shared<rclcpp::Node>("subscriber_node");
        auto subscription = subscriber_node->create_subscription<test_msgs::msg::Primitives>(
          "test_topic",
          [&messages, &counter](test_msgs::msg::Primitives::SharedPtr msg) {
            messages.push_back(msg->string_value);
            counter++;
          });
        while (counter < 1) {
          rclcpp::spin_some(subscriber_node);
        }
        return messages;
      });
  }
};


TEST_F(EndToEndTestFixture, messages_are_recorder_and_replayed_correctly) {
  auto publisher_future = publish_test_message();
  record_all_topics();
  publisher_future.wait();

  auto subscriber_future = subscribe_and_wait_for_one_test_message();
  play_bag();
  auto messages = subscriber_future.get();

  ASSERT_THAT(messages, Not(IsEmpty()));
  EXPECT_THAT(messages[0], Eq("test"));
}
