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
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/rosbag2.hpp"
#include "std_msgs/msg/string.hpp"

#include "../../src/rosbag2/storage/sqlite/sqlite_storage.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT

class Rosbag2TestFixture : public Test
{
public:
  Rosbag2TestFixture()
  : database_name_("test_database")
  {
    std::remove("test_database");
  }

  std::string database_name_;
};

std::vector<std::string> getMessagesFromSqliteDatabase(const std::string & database_name)
{
  sqlite::DBPtr db = sqlite::open(database_name);
  auto messages = sqlite::getMessages(db);
  sqlite::close(db);
  return messages;
}

TEST_F(Rosbag2TestFixture, published_messages_are_recorded)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  std::thread record_thread(
    [this]() {
      rosbag2::record(database_name_, "string_topic");
    });

  std::this_thread::sleep_for(std::chrono::seconds(1));

  auto node = std::make_shared<rclcpp::Node>("publisher_node");
  auto publisher = node->create_publisher<std_msgs::msg::String>("string_topic");

  auto msg = std_msgs::msg::String();
  msg.data = "Hello world";
  publisher->publish(msg);
  rclcpp::spin_some(node);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  rclcpp::shutdown();
  record_thread.join();

  auto messages = getMessagesFromSqliteDatabase(database_name_);

  ASSERT_THAT(messages, SizeIs(1));
  ASSERT_THAT(messages[0], Eq("Hello world"));
}
