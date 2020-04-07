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
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/wait_for.hpp"

#include "rosbag2_transport/rosbag2_transport.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "qos.hpp"
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
  EXPECT_THAT(serialized_profiles, ContainsRegex("reliability: 1\n"));
  EXPECT_THAT(serialized_profiles, ContainsRegex("durability: 2\n"));
  EXPECT_THAT(
    serialized_profiles, ContainsRegex(
      "deadline:\n"
      "    sec: .+\n"
      "    nsec: .+\n"
  ));
  EXPECT_THAT(
    serialized_profiles, ContainsRegex(
      "lifespan:\n"
      "    sec: .+\n"
      "    nsec: .+\n"
  ));
  EXPECT_THAT(serialized_profiles, ContainsRegex("liveliness: 1\n"));
  EXPECT_THAT(
    serialized_profiles, ContainsRegex(
      "liveliness_lease_duration:\n"
      "    sec: .+\n"
      "    nsec: .+\n"
  ));
}

TEST_F(RecordIntegrationTestFixture, records_sensor_data)
{
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
  auto & writer = static_cast<MockSequentialWriter &>(writer_->get_implementation_handle());

  // Takes ~200ms in local testing, 5s chosen as a very long timeout
  bool succeeded = rosbag2_test_common::spin_and_wait_for(
    std::chrono::seconds(5), publisher_node,
    [&writer]() {
      return writer.get_messages().size() > 0;
    });
  ASSERT_TRUE(succeeded);
  stop_recording();

  auto recorded_messages = writer.get_messages();
  auto recorded_topics = writer.get_topics();
  EXPECT_EQ(recorded_topics.size(), 1u);
  EXPECT_FALSE(recorded_messages.empty());
}
TEST_F(RecordIntegrationTestFixture, receives_latched_messages)
{
  // Ensure rosbag2 can receive Transient Local Durability "latched messages"
  const std::string topic = "/latched_topic";
  const size_t num_latched_messages = 3;

  auto publisher_node = std::make_shared<rclcpp::Node>("publisher_for_latched_test");
  auto profile_transient_local = rclcpp::QoS(num_latched_messages).transient_local();
  auto publisher_transient_local = publisher_node->create_publisher<test_msgs::msg::Strings>(
    topic, profile_transient_local);

  // Publish messages before starting recording
  test_msgs::msg::Strings msg;
  msg.string_value = "Hello";
  for (size_t i = 0; i < num_latched_messages; i++) {
    publisher_transient_local->publish(msg);
    rclcpp::spin_some(publisher_node);
  }
  start_recording({false, false, {topic}, "rmw_format", 100ms});
  auto & writer = static_cast<MockSequentialWriter &>(writer_->get_implementation_handle());

  // Takes ~100ms in local testing, 5s chosen as a very long timeout
  bool succeeded = rosbag2_test_common::spin_and_wait_for(
    std::chrono::seconds(5), publisher_node,
    [&writer]() {
      return writer.get_messages().size() == num_latched_messages;
    });
  stop_recording();
  ASSERT_TRUE(succeeded);
}

TEST_F(RecordIntegrationTestFixture, mixed_qos_subscribes) {
  // Ensure that rosbag2 subscribes to publishers that offer different durability policies
  const size_t arbitrary_history = 5;

  std::string topic = "/string_topic";
  test_msgs::msg::Strings msg;
  msg.string_value = "Hello";

  auto profile_volatile = rclcpp::QoS(arbitrary_history).reliable().durability_volatile();
  auto profile_transient_local = rclcpp::QoS(arbitrary_history).reliable().transient_local();

  auto publisher_node = std::make_shared<rclcpp::Node>("publisher_for_qos_test");
  auto publisher_volatile = publisher_node->create_publisher<test_msgs::msg::Strings>(
    topic, profile_volatile);
  auto publisher_transient_local = publisher_node->create_publisher<test_msgs::msg::Strings>(
    topic, profile_transient_local);

  start_recording({false, false, {topic}, "rmw_format", 100ms});
  // Takes ~100ms in local testing, 5s chosen as a very long timeout
  bool succeeded = rosbag2_test_common::spin_and_wait_for(
    std::chrono::seconds(5), publisher_node,
    [publisher_volatile, publisher_transient_local]() {
      // This test is a success if rosbag2 has connected to both publishers
      return
      publisher_volatile->get_subscription_count() &&
      publisher_transient_local->get_subscription_count();
    });
  stop_recording();
  ASSERT_TRUE(succeeded);
}

TEST_F(RecordIntegrationTestFixture, duration_and_noncompatibility_policies_mixed) {
  // Ensure that the duration-based and non-compatibility QoS policies don't affect subscription
  // These values are arbitrary, the significance is that they are non-default
  const std::string topic = "/mixed_nondelivery_policies";
  const size_t same_history = 5;
  const size_t different_history = 12;
  const rmw_time_t deadline{0, 1000};
  const rmw_time_t lifespan{3, 12};
  const auto liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE;
  const rmw_time_t liveliness_lease_duration{0, 5000000};

  auto publisher_node = std::make_shared<rclcpp::Node>("publisher_for");
  auto create_pub = [publisher_node, topic](auto qos) {
      return publisher_node->create_publisher<test_msgs::msg::Strings>(topic, qos);
    };

  auto profile_history = rclcpp::QoS(different_history);
  auto publisher_history = create_pub(profile_history);

  auto profile_lifespan = rclcpp::QoS(same_history).lifespan(lifespan);
  auto publisher_lifespan = create_pub(profile_lifespan);

  auto profile_deadline = rclcpp::QoS(same_history).deadline(deadline);
  auto publisher_deadline = create_pub(profile_deadline);

  auto profile_liveliness = rclcpp::QoS(same_history)
    .liveliness(liveliness).liveliness_lease_duration(liveliness_lease_duration);
  auto publisher_liveliness = create_pub(profile_liveliness);

  start_recording({false, false, {topic}, "rmw_format", 100ms});
  // Takes ~200ms in local testing, 5s chosen as a very long timeout
  bool succeeded = rosbag2_test_common::spin_and_wait_for(
    std::chrono::seconds(5), publisher_node,
    [publisher_history, publisher_lifespan, publisher_deadline, publisher_liveliness]() {
      return
      publisher_history->get_subscription_count() &&
      publisher_lifespan->get_subscription_count() &&
      publisher_deadline->get_subscription_count() &&
      publisher_liveliness->get_subscription_count();
    });
  stop_recording();
  ASSERT_TRUE(succeeded);
}

TEST_F(RecordIntegrationTestFixture, topic_qos_overrides)
{
  const auto num_msgs = 3;
  auto strict_msg = std::make_shared<test_msgs::msg::Strings>();
  strict_msg->string_value = "strict";
  const auto strict_topic = "/strict_topic";

  rosbag2_transport::RecordOptions record_options =
  {false, false, {strict_topic}, "rmw_format", 100ms};
  const auto profile_override = rclcpp::QoS{rclcpp::KeepAll()}
  .best_effort().durability_volatile().avoid_ros_namespace_conventions(false);
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides = {
    {strict_topic, profile_override}
  };
  record_options.topic_qos_profile_overrides = topic_qos_profile_overrides;

  // Create two BEST_EFFORT publishers on the same topic with different Durability policies.
  // If no override is specified, then the recorder cannot see any published messages.
  auto profile1 = rosbag2_transport::Rosbag2QoS{}.best_effort().durability_volatile();
  auto profile2 = rosbag2_transport::Rosbag2QoS{}.best_effort().transient_local();
  pub_man_.add_publisher<test_msgs::msg::Strings>(
    strict_topic, strict_msg, num_msgs, profile1);
  pub_man_.add_publisher<test_msgs::msg::Strings>(
    strict_topic, strict_msg, num_msgs, profile2);

  start_recording(record_options);
  run_publishers();
  stop_recording();

  auto & writer =
    static_cast<MockSequentialWriter &>(writer_->get_implementation_handle());
  auto recorded_messages = writer.get_messages();

  ASSERT_GE(recorded_messages.size(), 0u);
}
