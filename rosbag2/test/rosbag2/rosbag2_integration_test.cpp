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

#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/rosbag2.hpp"
#include "std_msgs/msg/string.hpp"

#include "../../src/rosbag2/storage/sqlite/sqlite_storage.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT
using namespace std::chrono_literals;

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

void recordMessage(
  std::string db_name, std::promise<void> * recorder_promise, std::promise<void> * rcl_promise)
{
  rosbag2::record(db_name, "string_topic", [&recorder_promise]() {
      static bool promise_already_set = false;
      if (!promise_already_set) {
        recorder_promise->set_value();
        promise_already_set = true;
      }
    });

  rcl_promise->set_value();
}

void publish_messages(std::shared_future<void> future)
{
  auto node = std::make_shared<rclcpp::Node>("publisher_node");
  auto publisher = node->create_publisher<std_msgs::msg::String>("string_topic");
  auto timer = node->create_wall_timer(500ms, [publisher]() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello world";
        publisher->publish(msg);
      });

  rclcpp::spin_until_future_complete(node, future);
}

TEST_F(Rosbag2TestFixture, published_messages_are_recorded)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  std::promise<void> recorder_promise;
  std::shared_future<void> recorder_future(recorder_promise.get_future());
  std::promise<void> rcl_safety_promise;
  std::future<void> rcl_safety_future = rcl_safety_promise.get_future();

  std::thread record_thread(recordMessage, database_name_, &recorder_promise, &rcl_safety_promise);
  publish_messages(recorder_future);

  recorder_future.wait();
  rclcpp::shutdown();
  record_thread.join();

  auto messages = getMessagesFromSqliteDatabase(database_name_);

  ASSERT_THAT(messages, Not(IsEmpty()));
  ASSERT_THAT(messages[0], Eq("Hello world"));

  rcl_safety_future.wait();
}
