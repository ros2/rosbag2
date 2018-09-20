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

#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_transport/rosbag2_transport.hpp"
#include "test_msgs/msg/primitives.hpp"
#include "test_msgs/msg/static_array_primitives.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

// TODO(Martin-Idel-SI): merge w. rosbag2_write_integration_test once signal handling is sorted out
class RosBag2IntegrationTestFixture : public Rosbag2TestFixture
{
public:
  RosBag2IntegrationTestFixture()
  : Rosbag2TestFixture()
  {
    rclcpp::init(0, nullptr);
    subscriber_node_ = std::make_shared<rclcpp::Node>("subscriber_node");
  }

  ~RosBag2IntegrationTestFixture() override
  {
    rclcpp::shutdown();
  }

  template<typename T>
  auto create_subscriber(const std::string & topic_name, size_t expected_number_of_messages)
  {
    rmw_qos_profile_t qos_profile;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    qos_profile.depth = 4;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    qos_profile.avoid_ros_namespace_conventions = false;
    expected_topics_with_size_[topic_name] = expected_number_of_messages;

    return subscriber_node_->create_subscription<T>(
      topic_name,
      [this, topic_name](std::shared_ptr<rcutils_char_array_t> msg) {
        subscribed_messages_[topic_name].push_back(msg);
      }, qos_profile);
  }

  std::future<void> start_spinning_subscriptions()
  {
    return async(
      std::launch::async, [this]() {
        while (continue_spinning(expected_topics_with_size_)) {
          rclcpp::spin_some(subscriber_node_);
        }
      });
  }

  bool continue_spinning(std::map<std::string, size_t> expected_topics_with_sizes)
  {
    for (const auto & topic_expected : expected_topics_with_sizes) {
      if (subscribed_messages_[topic_expected.first].size() < topic_expected.second) {
        return true;
      }
    }
    return false;
  }

  template<typename T>
  inline
  const rosidl_message_type_support_t * get_message_typesupport(std::shared_ptr<T>)
  {
    return rosidl_typesupport_cpp::get_message_type_support_handle<T>();
  }

  template<typename T>
  inline
  std::shared_ptr<T> deserialize_message(std::shared_ptr<rmw_serialized_message_t> serialized_msg)
  {
    auto message = std::make_shared<T>();
    auto error = rmw_deserialize(
      serialized_msg.get(),
      get_message_typesupport(message),
      message.get());
    if (error != RCL_RET_OK) {
      throw std::runtime_error("Failed to deserialize");
    }
    return message;
  }

  std::map<std::string, std::vector<std::shared_ptr<rcutils_char_array_t>>> subscribed_messages_;
  std::map<std::string, size_t> expected_topics_with_size_;
  rclcpp::Node::SharedPtr subscriber_node_;
};

TEST_F(RosBag2IntegrationTestFixture, recorded_messages_are_played_for_all_topics)
{
  auto primitive_message1 = get_messages_primitives()[0];
  primitive_message1->string_value = "Hello World 1";

  auto primitive_message2 = get_messages_primitives()[1];
  primitive_message2->string_value = "Hello World 2";

  auto complex_message1 = get_messages_static_array_primitives()[0];
  complex_message1->string_values = {{"Complex Hello1", "Complex Hello2", "Complex Hello3"}};
  complex_message1->bool_values = {{true, false, true}};

  auto complex_message2 = get_messages_static_array_primitives()[0];
  complex_message2->string_values = {{"Complex Hello4", "Complex Hello5", "Complex Hello6"}};
  complex_message2->bool_values = {{false, false, true}};

  auto topic_types = std::vector<rosbag2::TopicWithType>{
    {"topic1", "test_msgs/Primitives"},
    {"topic2", "test_msgs/StaticArrayPrimitives"},
  };

  std::vector<std::shared_ptr<rosbag2::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", primitive_message1),
    serialize_test_message("topic1", primitive_message2),
    serialize_test_message("topic1", primitive_message2),
    serialize_test_message("topic2", complex_message1),
    serialize_test_message("topic2", complex_message2),
    serialize_test_message("topic2", complex_message2)};

  reader_->prepare(messages, topic_types);

  // Due to a problem related to the subscriber, we play many (3) messages but make the subscriber
  // node spin only until 2 have arrived. Hence the 2 as `launch_subscriber()` argument.
  auto primitive_subscription = create_subscriber<test_msgs::msg::Primitives>("/topic1", 2);
  auto array_subscription = create_subscriber<test_msgs::msg::StaticArrayPrimitives>(
    "/topic2", 2);

  auto await_received_messages = start_spinning_subscriptions();


  Rosbag2Transport rosbag2_transport(reader_, writer_);
  rosbag2_transport.play(storage_options_, play_options_);

  await_received_messages.get();

  auto replayed_serialized_primitives = subscribed_messages_["/topic1"];
  std::vector<test_msgs::msg::Primitives::SharedPtr> replayed_test_primitives;
  for (auto & replayed_primitive : replayed_serialized_primitives) {
    replayed_test_primitives.push_back(
      deserialize_message<test_msgs::msg::Primitives>(replayed_primitive));
  }
  ASSERT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));
  EXPECT_THAT(replayed_test_primitives[0]->string_value, Eq(primitive_message1->string_value));
  EXPECT_THAT(replayed_test_primitives[1]->string_value, Eq(primitive_message2->string_value));

  auto replayed_serialized_arrays = subscribed_messages_["/topic2"];
  std::vector<test_msgs::msg::StaticArrayPrimitives::SharedPtr> replayed_test_arrays;
  for (auto & replayed_array : replayed_serialized_arrays) {
    replayed_test_arrays.push_back(
      deserialize_message<test_msgs::msg::StaticArrayPrimitives>(replayed_array));
  }
  ASSERT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
  EXPECT_THAT(replayed_test_arrays[0]->bool_values, Eq(complex_message1->bool_values));
  EXPECT_THAT(replayed_test_arrays[0]->string_values, Eq(complex_message1->string_values));
  EXPECT_THAT(replayed_test_arrays[1]->bool_values, Eq(complex_message2->bool_values));
  EXPECT_THAT(replayed_test_arrays[1]->string_values, Eq(complex_message2->string_values));
}
