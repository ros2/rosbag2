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
  : Rosbag2TestFixture(), message_received_(false)
  {}

  std::vector<std::string> subscribe_messages(std::shared_future<void> future)
  {
    std::vector<std::string> messages;
    auto node = std::make_shared<rclcpp::Node>("subscriber_node");
    auto subscription = node->create_subscription<std_msgs::msg::String>("string_topic",
        [&messages](const std_msgs::msg::String::ConstSharedPtr message)
        {messages.emplace_back(message->data);});

    future.wait();  // this starts any deferred thread
    rclcpp::spin_until_future_complete(node, future);
    return messages;
  }

  std::atomic<bool> message_received_;
};

TEST_F(RosBag2IntegrationTestFixture, recorded_messages_are_played)
{
  rclcpp::init(0, nullptr);

  std::vector<std::string> messages = {"Hello World", "Hello World!"};
  write_messages(database_name_, messages);

  rosbag2::Rosbag2 rosbag2;
  // Defer the future until we start subscribing
  auto future = std::async(
    std::launch::deferred, [this, &rosbag2]() {
      rosbag2.play(database_name_, "string_topic");
    });
  subscribe_messages(std::shared_future<void>(std::move(future)));
  rclcpp::shutdown();

  ASSERT_THAT(messages, SizeIs(2));
  ASSERT_THAT(messages[0], Eq("Hello World"));
  ASSERT_THAT(messages[1], Eq("Hello World!"));
}
