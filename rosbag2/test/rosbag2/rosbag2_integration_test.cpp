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
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/rosbag2.hpp"
#include "std_msgs/msg/string.hpp"

#include "rosbag2_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class RosBag2IntegrationFixture : public Test
{
public:
  RosBag2IntegrationFixture()
  {
    node_ = std::make_shared<rclcpp::Node>("rosbag2");
    rosbag2_ = rosbag2::Rosbag2();
  }

  void SetUp() override
  {
    auto publisher_node = std::make_shared<rclcpp::Node>("publisher_node");
    auto publisher = publisher_node->create_publisher<std_msgs::msg::String>("string_topic");
    future_ = std::async(std::launch::async, [publisher] {
          auto message = std_msgs::msg::String();
          message.data = "string message";
          publisher->publish(message);
        });
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void TearDown() override
  {
    future_.get();
  }

  rosbag2::Rosbag2 rosbag2_;
  std::shared_ptr<rclcpp::Node> node_;
  std::future<void> future_;
};

TEST_F(RosBag2IntegrationFixture,
  wait_for_topic_returns_with_topic_string_if_topic_is_specified_without_slash)
{
  std::string type = rosbag2_.wait_for_topic("string_topic", node_);

  EXPECT_THAT(type, StrEq("std_msgs/String"));
}

TEST_F(RosBag2IntegrationFixture,
  wait_for_topic_returns_with_topic_string_if_topic_is_specified_with_slash)
{
  std::string type = rosbag2_.wait_for_topic("/string_topic", node_);

  EXPECT_THAT(type, StrEq("std_msgs/String"));
}
