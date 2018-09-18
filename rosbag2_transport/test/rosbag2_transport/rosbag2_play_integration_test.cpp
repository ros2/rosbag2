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
  : Rosbag2TestFixture(), messages_stored_counter_(0)
  {
    rclcpp::init(0, nullptr);
  }

  ~RosBag2IntegrationTestFixture() override
  {
    rclcpp::shutdown();
  }

  template<typename T>
  auto prepare_subscriber(size_t expected_messages_number, const std::string & topic)
  {
    auto node = rclcpp::Node::make_shared("node_" + topic);
    auto messages = std::make_shared<std::vector<typename T::ConstSharedPtr>>();
    auto messages_received = std::make_shared<size_t>(0);
    rmw_qos_profile_t qos_profile;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    qos_profile.depth = 3;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    qos_profile.avoid_ros_namespace_conventions = false;

    auto subscription = node->create_subscription<T>(topic,
        [messages, messages_received](typename T::ConstSharedPtr message) {
          messages->push_back(message);
          ++*messages_received;
        }, qos_profile);
    subscriptions_.push_back(subscription);

    return [messages, messages_received, node, expected_messages_number]() {
             while (*messages_received < expected_messages_number) {
               rclcpp::spin_some(node);
             }
             return *messages;
           };
  }

  template<typename MessageT>
  auto launch_subscriber(size_t expected_messages_number, const std::string & topic)
  {
    auto spin_subscriber = prepare_subscriber<MessageT>(expected_messages_number, topic);
    return std::async(std::launch::async, spin_subscriber);
  }

  void wait_for_subscribers(size_t count)
  {
    std::async(std::launch::async, [this, count] {
        while (subscriptions_.size() < count) {
          std::this_thread::sleep_for(50ms);
        }
      }).get();
  }

  std::atomic<size_t> messages_stored_counter_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
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
  auto primitive_subscriber_future = launch_subscriber<test_msgs::msg::Primitives>(2, "topic1");
  auto static_array_subscriber_future =
    launch_subscriber<test_msgs::msg::StaticArrayPrimitives>(2, "topic2");
  wait_for_subscribers(2);

  Rosbag2Transport rosbag2_transport(reader_, writer_);
  rosbag2_transport.play(storage_options_, play_options_);

  auto replayed_test_primitives = primitive_subscriber_future.get();
  ASSERT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));
  EXPECT_THAT(replayed_test_primitives[0]->string_value, Eq(primitive_message1->string_value));
  EXPECT_THAT(replayed_test_primitives[1]->string_value, Eq(primitive_message2->string_value));

  auto replayed_test_arrays = static_array_subscriber_future.get();
  ASSERT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
  EXPECT_THAT(replayed_test_arrays[0]->bool_values, Eq(complex_message1->bool_values));
  EXPECT_THAT(replayed_test_arrays[0]->string_values, Eq(complex_message1->string_values));
  EXPECT_THAT(replayed_test_arrays[1]->bool_values, Eq(complex_message2->bool_values));
  EXPECT_THAT(replayed_test_arrays[1]->string_values, Eq(complex_message2->string_values));
}
