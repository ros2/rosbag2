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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <sqlite3.h>

#include <atomic>
#include <future>
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

class RosBag2IntegrationTestFixture : public Rosbag2TestFixture
{
public:
  RosBag2IntegrationTestFixture()
  : Rosbag2TestFixture(), message_received_(false)
  {}

  void recordMessage(std::string db_name)
  {
    rosbag2::Rosbag2 rosbag2;
    rosbag2.record(db_name, "string_topic", [this]() {
        message_received_ = true;
      });
  }

  void publish_messages()
  {
    auto node = std::make_shared<rclcpp::Node>("publisher_node");
    auto publisher = node->create_publisher<std_msgs::msg::String>("string_topic");
    auto timer = node->create_wall_timer(500ms, [publisher]() {
          auto msg = std_msgs::msg::String();
          msg.data = "Hello world";
          publisher->publish(msg);
        });

    while (!message_received_) {
      rclcpp::spin_some(node);
    }
  }

  void rclcppInit()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  std::atomic<bool> message_received_;
};

TEST_F(RosBag2IntegrationTestFixture, published_messages_are_recorded)
{
  rclcppInit();

  // the future object returned from std::async needs to be stored not to block the execution
  auto future = std::async(
    std::launch::async, [this]() {
      recordMessage(database_name_);
    });
  publish_messages();
  rclcpp::shutdown();

  sqlite3 * database;
  sqlite3_open(database_name_.c_str(), &database);
  auto messages = get_messages(database);
  sqlite3_close(database);

  ASSERT_THAT(messages, Not(IsEmpty()));
  ASSERT_THAT(messages[0], Eq("Hello world"));
}
