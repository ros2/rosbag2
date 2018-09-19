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

  void play_bag(const std::string & command)
  {
#ifdef _WIN32
    size_t length = strlen(command.c_str());
    TCHAR * command_char = new TCHAR[length + 1];
    memcpy(command_char, command.c_str(), length + 1);

    STARTUPINFO start_up_info{};
    PROCESS_INFORMATION process_info{};
    CreateProcess(
      nullptr,
      command_char,
      nullptr,
      nullptr,
      false,
      0,
      nullptr,
      database_path.c_str(),
      &start_up_info,
      &process_info);
    CloseHandle(process_info.hProcess);
    CloseHandle(process_info.hThread);
    delete[] command_char;
#else
    chdir(database_path_.c_str());
    system(command.c_str());
#endif
  }

  template<typename T>
  std::future<std::vector<std::shared_ptr<T>>> create_subscriber(
    const std::string & topic_name, size_t expected_number_of_messages)
  {
    return async(
      std::launch::async, [topic_name, expected_number_of_messages]() ->
      std::vector<std::shared_ptr<T>> {
        std::vector<std::shared_ptr<T>> messages;
        size_t counter = 0;
        rmw_qos_profile_t qos_profile;
        qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        qos_profile.depth = 4;
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
        qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
        qos_profile.avoid_ros_namespace_conventions = false;
        auto subscriber_node = std::make_shared<rclcpp::Node>(
          "subscriber_node" + topic_name.substr(1, topic_name.length()));
        auto subscription = subscriber_node->create_subscription<T>(
          topic_name,
          [&messages, &counter](std::shared_ptr<T> msg) {
            messages.push_back(msg);
            counter++;
          }, qos_profile);
        while (counter < expected_number_of_messages) {
          rclcpp::spin_some(subscriber_node);
        }
        return messages;
      });
  }

  std::string database_path_;
};

TEST_F(EndToEndTestFixture, play_end_to_end_test) {
  auto array_subscription = create_subscriber<test_msgs::msg::StaticArrayPrimitives>(
    "/array_topic", 2);
  auto primitive_subscription = create_subscriber<test_msgs::msg::Primitives>(
    "/test_topic", 3);
  play_bag("ros2 bag play test.bag");

  auto primitive_messages = primitive_subscription.get();
  auto array_messages = array_subscription.get();

  ASSERT_THAT(primitive_messages, SizeIs(Ge(3u)));
  EXPECT_THAT(primitive_messages[0]->string_value, Eq("test"));
  EXPECT_THAT(primitive_messages[1]->string_value, Eq("test"));
  EXPECT_THAT(primitive_messages[2]->string_value, Eq("test"));

  ASSERT_THAT(array_messages, SizeIs(Ge(2u)));
  EXPECT_THAT(array_messages[0]->bool_values, ElementsAre(true, false, true));
  EXPECT_THAT(array_messages[0]->string_values,
    ElementsAre("Complex Hello1", "Complex Hello2", "Complex Hello3"));
  EXPECT_THAT(array_messages[1]->bool_values, ElementsAre(true, false, true));
  EXPECT_THAT(array_messages[1]->string_values,
    ElementsAre("Complex Hello1", "Complex Hello2", "Complex Hello3"));
}
