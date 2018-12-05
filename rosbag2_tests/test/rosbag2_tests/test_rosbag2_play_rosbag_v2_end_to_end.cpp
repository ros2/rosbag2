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
#include <map>
#include <memory>
#include <string>
#include <vector>

// rclcpp must be included before process_execution_helpers.hpp
#include "rclcpp/rclcpp.hpp"
#include "process_execution_helpers.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "rosbag2_test_common/subscription_manager.hpp"

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

TEST_F(PlayEndToEndTestFixture, play_end_to_end_test_does_not_try_to_publish_ros1_topics) {
  sub_->add_subscription<std_msgs::msg::String>("/string_topic", 2);
  sub_->add_subscription<std_msgs::msg::Int32>("/int_topic", 2);

  auto subscription_future = sub_->spin_subscriptions();

  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag play test_bag.bag -s rosbag_v2", database_path_);

  subscription_future.get();

  auto string_messages = sub_->get_received_messages<std_msgs::msg::String>("/string_topic");
  auto int_messages = sub_->get_received_messages<std_msgs::msg::Int32>("/int_topic");

  EXPECT_THAT(string_messages, SizeIs(Ge(2u)));
  EXPECT_THAT(string_messages[0]->data, StrEq("foo"));

  EXPECT_THAT(int_messages, SizeIs(2));
  EXPECT_THAT(int_messages[0]->data, Eq(42));

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
}


TEST_F(PlayEndToEndTestFixture, play_fails_gracefully_if_rosbag_v2_storage_id_is_not_specified) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag play test_bag.bag", database_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  EXPECT_THAT(error_output, HasSubstr("No storage could be initialized"));
}
