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

#include "rclcpp/rclcpp.hpp"
#include "end_to_end_test_fixture.hpp"

#include "test_msgs/msg/primitives.hpp"
#include "test_msgs/msg/static_array_primitives.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"
#include "rosbag2_test_common/subscription_manager.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class PlayEndToEndTestFixture : public EndToEndTestFixture
{
public:
  PlayEndToEndTestFixture()
  {
    rclcpp::init(0, nullptr);
    sub_ = std::make_unique<SubscriptionManager>();
  }

  ~PlayEndToEndTestFixture() override
  {
    rclcpp::shutdown();
  }

  std::unique_ptr<SubscriptionManager> sub_;
};

TEST_F(PlayEndToEndTestFixture, play_end_to_end_test) {
  sub_->add_subscription<test_msgs::msg::StaticArrayPrimitives>("/array_topic", 2);
  sub_->add_subscription<test_msgs::msg::Primitives>("/test_topic", 3);

  auto subscription_future = sub_->spin_subscriptions();

  execute("ros2 bag play test");

  subscription_future.get();

  auto primitive_messages = sub_->get_received_messages<test_msgs::msg::Primitives>("/test_topic");
  auto array_messages = sub_->get_received_messages<test_msgs::msg::StaticArrayPrimitives>(
    "/array_topic");

  EXPECT_THAT(primitive_messages, SizeIs(Ge(3u)));
  EXPECT_THAT(primitive_messages,
    Each(Pointee(Field(&test_msgs::msg::Primitives::string_value, "test"))));

  EXPECT_THAT(array_messages, SizeIs(Ge(2u)));
  EXPECT_THAT(array_messages,
    Each(Pointee(Field(&test_msgs::msg::StaticArrayPrimitives::bool_values,
    ElementsAre(true, false, true)))));
  EXPECT_THAT(array_messages,
    Each(Pointee(Field(&test_msgs::msg::StaticArrayPrimitives::string_values,
    ElementsAre("Complex Hello1", "Complex Hello2", "Complex Hello3")))));
}
