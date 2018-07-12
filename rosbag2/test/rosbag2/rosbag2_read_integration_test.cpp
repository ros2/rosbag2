/*
 *  Copyright (c) 2018,  Bosch Software Innovations GmbH.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include <gmock/gmock.h>

#include <sqlite3.h>

#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <utility>

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
  : Rosbag2TestFixture(), counter_(0)
  {}

  std::vector<std::string> subscribe_messages()
  {
    std::vector<std::string> messages;
    auto node = std::make_shared<rclcpp::Node>("subscriber_node");
    auto subscription = node->create_subscription<std_msgs::msg::String>("string_topic",
        [&messages](const std_msgs::msg::String::ConstSharedPtr message)
        {messages.emplace_back(message->data);});

    while (counter_ == 0) {
      rclcpp::spin_some(node);
    }
    return messages;
  }

  void launch_subscriber()
  {
    subscriber_future_ = std::async(std::launch::async, [this] {return subscribe_messages();});
  }

  void play_bag(std::string database_name, std::string topic)
  {
    rosbag2::Rosbag2 rosbag2;
    rosbag2.play(database_name, topic);
    counter_++;
  }

  std::atomic<size_t> counter_;
  std::future<std::vector<std::string>> subscriber_future_;
};

TEST_F(RosBag2IntegrationTestFixture, recorded_messages_are_played)
{
  std::vector<std::string> messages = {"Hello World", "Hello World!"};
  write_messages(database_name_, messages);

  rclcpp::init(0, nullptr);
  launch_subscriber();
  play_bag(database_name_, "string_topic");
  rclcpp::shutdown();

  auto replayed_messages = subscriber_future_.get();
  ASSERT_THAT(replayed_messages, SizeIs(2));
  ASSERT_THAT(replayed_messages[0], Eq("Hello World"));
  ASSERT_THAT(replayed_messages[1], Eq("Hello World!"));
}
