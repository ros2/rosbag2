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

#include <memory>
#include <string>
#include <vector>

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "record_integration_fixture.hpp"

TEST_F(RecordIntegrationTestFixture, record_all_without_discovery_ignores_later_announced_topics)
{
  auto topic = "/string_topic";
  auto string_message = get_messages_strings()[0];
  string_message->string_value = "Hello World";

  auto publisher_node = std::make_shared<rclcpp::Node>(
    "publisher_for_test",
    rclcpp::NodeOptions().start_parameter_event_publisher(false));

  start_recording({true, true, {}, "rmw_format", 1ms});

  std::this_thread::sleep_for(100ms);
  auto publisher = publisher_node->create_publisher<test_msgs::msg::Strings>(topic, 10);
  for (int i = 0; i < 5; ++i) {
    std::this_thread::sleep_for(20ms);
    publisher->publish(*string_message);
  }
  stop_recording();

  MockSequentialWriter & writer =
    static_cast<MockSequentialWriter &>(writer_->get_implementation_handle());
  auto recorded_topics = writer.get_topics();
  EXPECT_EQ(0u, recorded_topics.count(topic));
}
