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

#include <future>
#include <iostream>
#include <map>
#include <memory>
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
    subscriber_node_ = std::make_shared<rclcpp::Node>("subscriber_node");
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
      database_path_.c_str(),
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

  void push_back(const std::string & topic_name, std::shared_ptr<rcutils_char_array_t> message)
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    subscribed_messages_[topic_name].push_back(message);
  }

  template<typename T>
  auto create_subscriber(const std::string & topic_name, size_t expected_number_of_messages)
  {
    rmw_qos_profile_t qos_profile;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    qos_profile.depth = 4;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    qos_profile.avoid_ros_namespace_conventions = false;
    expected_topics_with_size_[topic_name] = expected_number_of_messages;

    return subscriber_node_->create_subscription<T>(
      topic_name,
      [this, topic_name](std::shared_ptr<rcutils_char_array_t> msg) {
        this->push_back(topic_name, msg);
      }, qos_profile);
  }

  template<typename T>
  inline
  const rosidl_message_type_support_t * get_message_typesupport(std::shared_ptr<T>)
  {
    return rosidl_typesupport_cpp::get_message_type_support_handle<T>();
  }

  template<typename T>
  inline
  std::shared_ptr<T> deserialize_message(std::shared_ptr<rmw_serialized_message_t> serialized_msg)
  {
    auto message = std::make_shared<T>();
    auto error = rmw_deserialize(
      serialized_msg.get(),
      get_message_typesupport(message),
      message.get());
    if (error != RCL_RET_OK) {
      throw std::runtime_error("Failed to deserialize");
    }
    return message;
  }

  std::future<void> start_spinning_subscriptions()
  {
    return async(
      std::launch::async, [this]() {
        while (continue_spinning(expected_topics_with_size_)) {
          rclcpp::spin_some(subscriber_node_);
        }
      });
  }

  bool continue_spinning(std::map<std::string, size_t> expected_topics_with_sizes)
  {
    for (const auto & topic_expected : expected_topics_with_sizes) {
      if (subscribed_messages_[topic_expected.first].size() < topic_expected.second) {
        return true;
      }
    }
    return false;
  }

  std::mutex write_mutex_;
  std::map<std::string, std::vector<std::shared_ptr<rcutils_char_array_t>>> subscribed_messages_;
  std::map<std::string, size_t> expected_topics_with_size_;
  std::string database_path_;
  rclcpp::Node::SharedPtr subscriber_node_;
};

TEST_F(EndToEndTestFixture, play_end_to_end_test) {
  auto array_subscription = create_subscriber<test_msgs::msg::StaticArrayPrimitives>(
    "/array_topic", 2);
  auto primitive_subscription = create_subscriber<test_msgs::msg::Primitives>("/test_topic", 3);

  auto future = start_spinning_subscriptions();

  play_bag("ros2 bag play test.bag");

  future.get();

  auto primitive_messages = subscribed_messages_["/test_topic"];
  auto array_messages = subscribed_messages_["/array_topic"];

  ASSERT_THAT(primitive_messages, SizeIs(Ge(3u)));
  EXPECT_THAT(deserialize_message<test_msgs::msg::Primitives>(primitive_messages[0])->string_value,
    Eq("test"));
  EXPECT_THAT(deserialize_message<test_msgs::msg::Primitives>(primitive_messages[1])->string_value,
    Eq("test"));
  EXPECT_THAT(deserialize_message<test_msgs::msg::Primitives>(primitive_messages[2])->string_value,
    Eq("test"));

  ASSERT_THAT(array_messages, SizeIs(Ge(2u)));
  auto first_array_message = deserialize_message<test_msgs::msg::StaticArrayPrimitives>(
    array_messages[0]);
  EXPECT_THAT(first_array_message->bool_values, ElementsAre(true, false, true));
  EXPECT_THAT(first_array_message->string_values,
    ElementsAre("Complex Hello1", "Complex Hello2", "Complex Hello3"));
  auto second_array_message = deserialize_message<test_msgs::msg::StaticArrayPrimitives>(
    array_messages[1]);
  EXPECT_THAT(second_array_message->bool_values, ElementsAre(true, false, true));
  EXPECT_THAT(second_array_message->string_values,
    ElementsAre("Complex Hello1", "Complex Hello2", "Complex Hello3"));
}
