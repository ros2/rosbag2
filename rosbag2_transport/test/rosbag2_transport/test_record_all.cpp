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

#include "rclcpp/rclcpp.hpp"
#include "record_integration_fixture.hpp"
#include "rosbag2_transport/rosbag2_transport.hpp"
#include "rosbag2/types.hpp"
#include "test_msgs/msg/primitives.hpp"
#include "test_msgs/msg/static_array_primitives.hpp"
#include "test_msgs/message_fixtures.hpp"

TEST_F(RecordIntegrationTestFixture, published_messages_from_multiple_topics_are_recorded)
{
  auto array_message = get_messages_static_array_primitives()[0];
  array_message->string_values = {{"Complex Hello1", "Complex Hello2", "Complex Hello3"}};
  array_message->bool_values = {{true, false, true}};
  std::string array_topic = "/array_topic";

  auto string_message = get_messages_primitives()[0];
  string_message->string_value = "Hello World";
  std::string string_topic = "/string_topic";

  pub_man_.add_publisher<test_msgs::msg::Primitives>(string_topic, string_message, 2);
  pub_man_.add_publisher<test_msgs::msg::StaticArrayPrimitives>(array_topic, array_message, 2);

  start_recording({true, {}, "", 100ms});
  run_publishers();
  stop_recording();

  auto recorded_messages = writer_->get_messages();

  ASSERT_THAT(recorded_messages, SizeIs(4));
  auto string_messages = filter_messages<test_msgs::msg::Primitives>(
    recorded_messages, string_topic);
  auto array_messages = filter_messages<test_msgs::msg::StaticArrayPrimitives>(
    recorded_messages, array_topic);
  ASSERT_THAT(string_messages, SizeIs(2));
  ASSERT_THAT(array_messages, SizeIs(2));
  EXPECT_THAT(string_messages[0]->string_value, Eq("Hello World"));
  EXPECT_THAT(array_messages[0]->bool_values, ElementsAre(true, false, true));
  EXPECT_THAT(array_messages[0]->string_values,
    ElementsAre("Complex Hello1", "Complex Hello2", "Complex Hello3"));
}
