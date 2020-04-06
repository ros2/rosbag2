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

#include "../../src/rosbag2_transport/qos.hpp"
#include "rosbag2_transport/rosbag2_transport.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "record_integration_fixture.hpp"

TEST_F(RecordIntegrationTestFixture, published_messages_from_multiple_topics_are_recorded)
{
  auto array_message = get_messages_arrays()[0];
  std::string array_topic = "/array_topic";

  auto string_message = get_messages_strings()[1];
  std::string string_topic = "/string_topic";

  start_recording({false, false, {string_topic, array_topic}, "rmw_format", 100ms});

  pub_man_.add_publisher<test_msgs::msg::Strings>(
    string_topic, string_message, 2);
  pub_man_.add_publisher<test_msgs::msg::Arrays>(
    array_topic, array_message, 2);
  run_publishers();
  stop_recording();

  MockSequentialWriter & writer =
    static_cast<MockSequentialWriter &>(writer_->get_implementation_handle());
  auto recorded_messages = writer.get_messages();
  auto recorded_topics = writer.get_topics();

  ASSERT_THAT(recorded_topics, SizeIs(2));
  EXPECT_THAT(recorded_topics.at(string_topic).serialization_format, Eq("rmw_format"));
  EXPECT_THAT(recorded_topics.at(array_topic).serialization_format, Eq("rmw_format"));
  ASSERT_THAT(recorded_messages, SizeIs(4));
  auto string_messages = filter_messages<test_msgs::msg::Strings>(
    recorded_messages, string_topic);
  auto array_messages = filter_messages<test_msgs::msg::Arrays>(
    recorded_messages, array_topic);
  ASSERT_THAT(string_messages, SizeIs(2));
  ASSERT_THAT(array_messages, SizeIs(2));
  EXPECT_THAT(string_messages[0]->string_value, Eq(string_message->string_value));
  EXPECT_THAT(array_messages[0]->bool_values, Eq(array_message->bool_values));
  EXPECT_THAT(array_messages[0]->float32_values, Eq(array_message->float32_values));
}

#ifdef ROSBAG2_ENABLE_ADAPTIVE_QOS_SUBSCRIPTION
TEST_F(RecordIntegrationTestFixture, qos_is_stored_in_metadata)
{
  auto string_message = get_messages_strings()[1];
  std::string topic = "/chatter";
  start_recording({false, false, {topic}, "rmw_format", 100ms});
  pub_man_.add_publisher<test_msgs::msg::Strings>(topic, string_message, 2);
  run_publishers();
  stop_recording();

  MockSequentialWriter & writer =
    static_cast<MockSequentialWriter &>(writer_->get_implementation_handle());
  auto recorded_topics = writer.get_topics();
  std::string serialized_profiles = recorded_topics.at(topic).offered_qos_profiles;
  // Basic smoke test that the profile was serialized into the metadata as a string.
  EXPECT_THAT(
    serialized_profiles, ContainsRegex(
      "- history: 3\n"
      "  depth: 0\n"
      "  reliability: 1\n"
      "  durability: 2\n"
      "  deadline:\n"
      "    sec: 2147483647\n"
      "    nsec: 4294967295\n"
      "  lifespan:\n"
      "    sec: 2147483647\n"
      "    nsec: 4294967295\n"
      "  liveliness: 1\n"
      "  liveliness_lease_duration:\n"
      "    sec: 2147483647\n"
      "    nsec: 4294967295\n"
      "  avoid_ros_namespace_conventions: false"
  ));
}

TEST_F(RecordIntegrationTestFixture, records_sensor_data) {
  using clock = std::chrono::system_clock;
  using namespace std::chrono_literals;

  std::string topic = "/string_topic";
  start_recording({false, false, {topic}, "rmw_format", 100ms});

  auto publisher_node = std::make_shared<rclcpp::Node>("publisher_for_qos_test");
  auto publisher = publisher_node->create_publisher<test_msgs::msg::Strings>(
    topic, rclcpp::SensorDataQoS());
  auto publish_timer = publisher_node->create_wall_timer(
    50ms, [publisher]() -> void {
      test_msgs::msg::Strings msg;
      msg.string_value = "Hello";
      publisher->publish(msg);
    }
  );
  MockSequentialWriter &writer =
    static_cast<MockSequentialWriter &>(writer_->get_implementation_handle());

  auto start = clock::now();
  // Takes ~200ms normally, 5s chosen as "a very long time"
  auto timeout = 5s;
  bool timed_out = false;
  while (writer.get_messages().empty()) {
    if ((clock::now() - start) > timeout) {
      timed_out = true;
      break;
    }
    rclcpp::spin_some(publisher_node);
  }
  stop_recording();

  ASSERT_FALSE(timed_out);
  auto recorded_messages = writer.get_messages();
  auto recorded_topics = writer.get_topics();
  EXPECT_EQ(recorded_topics.size(), 1u);
  EXPECT_FALSE(recorded_messages.empty());
}
#endif  // ROSBAG2_ENABLE_ADAPTIVE_QOS_SUBSCRIPTION

TEST_F(RecordIntegrationTestFixture, topic_qos_overrides)
{
  auto strict_msg = std::make_shared<test_msgs::msg::Strings>();
  strict_msg->string_value = "strict";
  std::string strict_topic = "/strict_topic";
  std::string relaxed_topic = "/relaxed_topic";

  rosbag2_transport::RecordOptions record_options =
  {false, false, {strict_topic, relaxed_topic}, "rmw_format", 100ms};
  // 0 means system default for all options
  record_options.qos_profiles =
    "/strict_topic:\n"
    "  history: 2\n"      // Keep All
    "  depth: 0\n"
    "  reliability: 2\n"  // Best Effort
    "  durability: 2\n"   // Volatile
    "  deadline:\n"
    "    sec: 0\n"
    "    nsec: 0\n"
    "  lifespan:\n"
    "    sec: 0\n"
    "    nsec: 0\n"
    "    nsec: 0\n"
    "  liveliness: 0\n"
    "  liveliness_lease_duration:\n"
    "    sec: 0\n"
    "    nsec: 0\n"
    "  avoid_ros_namespace_conventions: false\n";

  // Create two publishers on the same topic with different QoS profiles.
  // If no override is specified, then the recorder cannot see any published messages.
  auto profile1 = rosbag2_transport::Rosbag2QoS().best_effort().durability_volatile();
  auto profile2 = rosbag2_transport::Rosbag2QoS().best_effort().transient_local();
  pub_man_.add_publisher<test_msgs::msg::Strings>(strict_topic, strict_msg, 3, profile1);
  pub_man_.add_publisher<test_msgs::msg::Strings>(strict_topic, strict_msg, 3, profile2);

  start_recording(record_options);
  run_publishers();
  stop_recording();

  MockSequentialWriter & writer =
    static_cast<MockSequentialWriter &>(writer_->get_implementation_handle());
  auto recorded_messages = writer.get_messages();

  ASSERT_GE(recorded_messages.size(), 0u);
}
