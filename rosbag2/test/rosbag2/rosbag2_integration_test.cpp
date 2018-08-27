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
    publisher_node_ = std::make_shared<rclcpp::Node>("publisher_node");
    rosbag2_ = rosbag2::Rosbag2();
  }

  std::future<void> create_publisher(std::string topic)
  {
    auto publisher = publisher_node_->create_publisher<std_msgs::msg::String>(topic);
    return std::async(std::launch::async, [publisher] {
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

  rosbag2::Rosbag2 rosbag2_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
};

TEST_F(RosBag2IntegrationFixture,
  get_topic_returns_with_topic_string_if_topic_is_specified_without_slash)
{
  auto future = create_publisher("string_topic");

  auto topics_and_types = rosbag2_.get_topics_with_types({"string_topic"}, node_);

  EXPECT_THAT(topics_and_types, SizeIs(1));
  EXPECT_THAT(topics_and_types.begin()->second, StrEq("std_msgs/String"));
  future.get();
}

TEST_F(RosBag2IntegrationFixture,
  get_topic_returns_with_topic_string_if_topic_is_specified_with_slash)
{
  auto future = create_publisher("string_topic");

  auto topics_and_types = rosbag2_.get_topics_with_types({"/string_topic"}, node_);

  EXPECT_THAT(topics_and_types, SizeIs(1));
  EXPECT_THAT(topics_and_types.begin()->second, StrEq("std_msgs/String"));
  future.get();
}

TEST_F(RosBag2IntegrationFixture,
  returns_multiple_topics_for_multiple_inputs)
{
  std::string first_topic("/string_topic");
  std::string second_topic("/other_topic");
  std::string third_topic("/wrong_topic");

  auto future1 = create_publisher(first_topic);
  auto future2 = create_publisher(second_topic);
  auto future3 = create_publisher(third_topic);

  auto topics_and_types = rosbag2_.get_topics_with_types({first_topic, second_topic}, node_);

  EXPECT_THAT(topics_and_types, SizeIs(2));
  EXPECT_THAT(topics_and_types.find(first_topic)->second, StrEq("std_msgs/String"));
  EXPECT_THAT(topics_and_types.find(second_topic)->second, StrEq("std_msgs/String"));
  future1.get();
}

TEST_F(RosBag2IntegrationFixture, get_topic_returns_empty_if_topic_does_not_exist) {
  auto future = create_publisher("string_topic");

  auto topics_and_types = rosbag2_.get_topics_with_types({"/wrong_topic"}, node_);

  EXPECT_THAT(topics_and_types, IsEmpty());
  future.get();
}
