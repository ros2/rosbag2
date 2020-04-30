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

#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// rclcpp must be included before process_execution_helpers.hpp
#include "rclcpp/rclcpp.hpp"

#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"

#include "rosbag2_test_common/subscription_manager.hpp"
#include "rosbag2_test_common/process_execution_helpers.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class PlayEndToEndTestFixture : public Test
{
public:
  PlayEndToEndTestFixture()
  {
    database_path_ = _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt
    sub_ = std::make_unique<SubscriptionManager>();
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  std::string database_path_;
  std::unique_ptr<SubscriptionManager> sub_;
};

TEST_F(PlayEndToEndTestFixture, play_end_to_end_test) {
  sub_->add_subscription<test_msgs::msg::Arrays>("/array_topic", 2);
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/test_topic", 3);

  auto subscription_future = sub_->spin_subscriptions();

  auto exit_code = execute_and_wait_until_completion("ros2 bag play cdr_test", database_path_);

  subscription_future.get();

  auto primitive_messages = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/test_topic");
  auto array_messages = sub_->get_received_messages<test_msgs::msg::Arrays>("/array_topic");

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));

  EXPECT_THAT(primitive_messages, SizeIs(Ge(3u)));
  EXPECT_THAT(
    primitive_messages,
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int32_value, 123))));

  EXPECT_THAT(array_messages, SizeIs(Ge(2u)));
  EXPECT_THAT(
    array_messages,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::bool_values,
          ElementsAre(true, false, true)))));
  EXPECT_THAT(
    array_messages,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::string_values,
          ElementsAre("Complex Hello1", "Complex Hello2", "Complex Hello3")))));
}

TEST_F(PlayEndToEndTestFixture, play_fails_gracefully_if_bag_does_not_exist) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag play does_not_exist", database_path_);
  auto error_output = internal::GetCapturedStderr();

  // Exit code could be EXIT_FAILURE (1) or 2 (no such file or directory)
  EXPECT_THAT(exit_code, Ne(EXIT_SUCCESS));
  EXPECT_THAT(error_output, HasSubstr("'does_not_exist' does not exist"));
}

TEST_F(PlayEndToEndTestFixture, play_fails_gracefully_if_needed_coverter_plugin_does_not_exist) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag play wrong_rmw_test", database_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  EXPECT_THAT(
    error_output, HasSubstr("Requested converter for format 'wrong_format' does not exist"));
}

TEST_F(PlayEndToEndTestFixture, play_filters_by_topic) {
  // Play a specific topic
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/test_topic", 2);  // 3);
  sub_->add_subscription<test_msgs::msg::Arrays>("/array_topic", 0);

  auto subscription_future = sub_->spin_subscriptions();

  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag play cdr_test --topics /test_topic",
    database_path_);

  subscription_future.get();

  auto primitive_messages = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/test_topic");
  auto array_messages = sub_->get_received_messages<test_msgs::msg::Arrays>("/array_topic");

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));

  EXPECT_THAT(primitive_messages, SizeIs(Ge(2u)));  // 3u)));
  EXPECT_THAT(array_messages, SizeIs(Ge(0u)));

  // Play a different topic
  sub_ = std::make_unique<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/test_topic", 0);
  sub_->add_subscription<test_msgs::msg::Arrays>("/array_topic", 1);  // 2);

  subscription_future = sub_->spin_subscriptions();

  exit_code = execute_and_wait_until_completion(
    "ros2 bag play --topics /array_topic -- cdr_test",
    database_path_);

  subscription_future.get();

  primitive_messages = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/test_topic");
  array_messages = sub_->get_received_messages<test_msgs::msg::Arrays>("/array_topic");

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));

  EXPECT_THAT(primitive_messages, SizeIs(Ge(0u)));
  EXPECT_THAT(array_messages, SizeIs(Ge(1u)));  // 2u)));

  // Play all topics
  sub_ = std::make_unique<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/test_topic", 2);  // 3);
  sub_->add_subscription<test_msgs::msg::Arrays>("/array_topic", 1);  // 2);

  subscription_future = sub_->spin_subscriptions();

  exit_code = execute_and_wait_until_completion(
    "ros2 bag play --topics /test_topic /array_topic -- cdr_test",
    database_path_);

  subscription_future.get();

  primitive_messages = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/test_topic");
  array_messages = sub_->get_received_messages<test_msgs::msg::Arrays>("/array_topic");

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));

  EXPECT_THAT(primitive_messages, SizeIs(Ge(2u)));  // 3u)));
  EXPECT_THAT(array_messages, SizeIs(Ge(1u)));  // 2u)));

  // Play a non-existent topic
  sub_ = std::make_unique<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/test_topic", 0);
  sub_->add_subscription<test_msgs::msg::Arrays>("/array_topic", 0);

  subscription_future = sub_->spin_subscriptions();

  exit_code = execute_and_wait_until_completion(
    "ros2 bag play --topics /nonexistent_topic -- cdr_test", database_path_);

  subscription_future.get();

  primitive_messages = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/test_topic");
  array_messages = sub_->get_received_messages<test_msgs::msg::Arrays>("/array_topic");

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));

  EXPECT_THAT(primitive_messages, SizeIs(Ge(0u)));
  EXPECT_THAT(array_messages, SizeIs(Ge(0u)));
}
