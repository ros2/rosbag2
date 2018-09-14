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
#include "std_msgs/msg/string.hpp"

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
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    kill(-process_id, SIGKILL);
  }

  void play_bag()
  {
    system("ros2 bag play test.bag");
  }

  void publish_string_message()
  {
    auto publisher_node = std::make_shared<rclcpp::Node>("publisher_node");
    auto publisher = publisher_node->create_publisher<std_msgs::msg::String>("test_topic");
    for (int i = 0; i < 10; i++) {
      std_msgs::msg::String string_message;
      string_message.data = "test";
      publisher->publish(string_message);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  std::vector<std::string> subscribe_to_string_topic()
  {
    std::vector<std::string> messages;
    int counter = 0;
    auto subscriber_node = std::make_shared<rclcpp::Node>("subscriber_node");
    auto subscription = subscriber_node->create_subscription<std_msgs::msg::String>(
      "test_topic",
      [&messages, &counter](std_msgs::msg::String::SharedPtr msg) {
        messages.push_back(msg->data);
        counter++;
      });
    while (counter < 1) {
      rclcpp::spin_some(subscriber_node);
    }
    return messages;
  }
};


TEST_F(EndToEndTestFixture, messages_are_recorder_and_replayed_correctly) {
  auto publisher_future = async(std::launch::async, [this]() {publish_string_message();});
  record_all_topics();
  publisher_future.wait();

  auto subscriber_future = async(std::launch::async, [this]() -> std::vector<std::string>
      {
        return subscribe_to_string_topic();
      }
  );
  play_bag();
  auto messages = subscriber_future.get();

  ASSERT_THAT(messages, Not(IsEmpty()));
  EXPECT_THAT(messages[0], Eq("test"));
}
