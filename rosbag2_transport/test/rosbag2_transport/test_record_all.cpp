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

#include <string>
#include<utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/wait_for.hpp"

#include "rosbag2_transport/recorder.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "record_integration_fixture.hpp"

TEST_F(RecordIntegrationTestFixture, published_messages_from_multiple_topics_are_recorded)
{
  auto array_message = get_messages_arrays()[0];
  array_message->float32_values = {{40.0f, 2.0f, 0.0f}};
  array_message->bool_values = {{true, false, true}};
  std::string array_topic = "/array_topic";

  auto string_message = get_messages_strings()[0];
  string_message->string_value = "Hello World";
  std::string string_topic = "/string_topic";

  // TODO(karsten1987) Refactor this into publication manager
  auto pub_node = std::make_shared<rclcpp::Node>("rosbag2_record_all_test_node");
  auto array_pub = pub_node->create_publisher<test_msgs::msg::Arrays>(
    array_topic, rclcpp::QoS{rclcpp::KeepAll()});
  auto string_pub = pub_node->create_publisher<test_msgs::msg::Strings>(
    string_topic, rclcpp::QoS{rclcpp::KeepAll()});

  rosbag2_transport::RecordOptions record_options = {true, false, {}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  for (auto i = 0u; i < 2; ++i) {
    array_pub->publish(*array_message);
  }
  for (auto i = 0u; i < 2; ++i) {
    string_pub->publish(*string_message);
  }

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = 4;
  auto ret = rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(5),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() >= expected_messages;
    });
  auto recorded_messages = mock_writer.get_messages();

  // We may receive additional messages from rosout, it doesn't matter,
  // as long as we have received at least as many total messages as we expect
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(Ge(expected_messages)));

  auto string_messages = filter_messages<test_msgs::msg::Strings>(
    recorded_messages, string_topic);
  auto array_messages = filter_messages<test_msgs::msg::Arrays>(
    recorded_messages, array_topic);
  ASSERT_THAT(string_messages, SizeIs(2));
  ASSERT_THAT(array_messages, SizeIs(2));
  EXPECT_THAT(string_messages[0]->string_value, Eq("Hello World"));
  EXPECT_THAT(array_messages[0]->bool_values, ElementsAre(true, false, true));
  EXPECT_THAT(array_messages[0]->float32_values, ElementsAre(40.0f, 2.0f, 0.0f));
}
