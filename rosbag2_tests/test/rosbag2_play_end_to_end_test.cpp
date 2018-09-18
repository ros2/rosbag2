/*
 *  Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#include <iostream>
#include <memory>
#include <future>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"  // rclcpp must be included before the Windows specific includes.

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#endif

#include "test_msgs/msg/primitives.hpp"
#include "test_msgs/msg/static_array_primitives.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT


class EndToEndTestFixture : public Test
{
public:
  EndToEndTestFixture()
  {
    database_path_ = _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt
    rclcpp::init(0, nullptr);
  }

  ~EndToEndTestFixture() override
  {
    rclcpp::shutdown();
  }

  void play_bag()
  {
#ifdef _WIN32
    STARTUPINFO start_up_info{};
    PROCESS_INFORMATION process_info{};
    CreateProcess(
      nullptr,
      "ros2 bag play test.bag",
      nullptr,
      nullptr,
      false,
      0,
      nullptr,
      database_path.c_str(),
      &start_up_info,
      &process_info);
#else
    chdir(database_path_.c_str());
    system("ros2 bag play test.bag");
#endif
  }

  template<typename T>
  std::future<std::vector<std::shared_ptr<T>>> subscribe_and_wait_for_messages(
    const std::string & topic_name, size_t expected_number_of_messages)
  {
    (void) expected_number_of_messages;
    return async(
      std::launch::async, [topic_name]() ->
      std::vector<std::shared_ptr<T>> {
        std::vector<std::shared_ptr<T>> messages;
        size_t counter = 0;
        rmw_qos_profile_t qos_profile;
        qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        qos_profile.depth = 4;
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
        qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
        qos_profile.avoid_ros_namespace_conventions = false;
        static int node_count = 0;
        auto subscriber_node = std::make_shared<rclcpp::Node>(
          "subscriber_node" + std::to_string(node_count++));
        auto subscription = subscriber_node->create_subscription<T>(
          topic_name,
          [&messages, &counter](std::shared_ptr<T> msg) {
            messages.push_back(msg);
            counter++;
          }, qos_profile);
        while (counter < 2) {
          rclcpp::spin_some(subscriber_node);
        }
        return messages;
      });
  }

  std::string database_path_;
};

TEST_F(EndToEndTestFixture, play_end_to_end_test) {

  auto array_subscriber = subscribe_and_wait_for_messages<test_msgs::msg::StaticArrayPrimitives>(
    "/array_topic", 3);
  auto string_subscriber = subscribe_and_wait_for_messages<test_msgs::msg::Primitives>(
    "/test_topic", 4);
  play_bag();

  auto string_messages = string_subscriber.get();
  auto array_messages = array_subscriber.get();

  ASSERT_THAT(string_messages, SizeIs(1));
  EXPECT_THAT(string_messages[0]->string_value, Eq("test"));

  ASSERT_THAT(array_messages, SizeIs(1));
  EXPECT_THAT(array_messages[0]->bool_values, ElementsAre(true, false, true));
  EXPECT_THAT(array_messages[0]->string_values,
    ElementsAre("Complex Hello1", "Complex Hello2", "Complex Hello3"));
}