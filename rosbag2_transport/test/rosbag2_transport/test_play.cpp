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

#include "rosbag2_transport_test_fixture.hpp"
#include "subscription_manager.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class RosBag2PlayTestFixture : public Rosbag2TransportTestFixture
{
public:
  RosBag2PlayTestFixture()
  : Rosbag2TransportTestFixture()
  {
    rclcpp::init(0, nullptr);
    sub_ = std::make_shared<test_helpers::SubscriptionManager>();
  }

  ~RosBag2PlayTestFixture() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<test_helpers::SubscriptionManager> sub_;
};

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_all_topics)
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
  sub_->add_subscription<test_msgs::msg::Primitives>("/topic1", 2);
  sub_->add_subscription<test_msgs::msg::StaticArrayPrimitives>(
    "/topic2", 2);

  auto await_received_messages = sub_->spin_subscriptions();

  Rosbag2Transport rosbag2_transport(reader_, writer_);
  rosbag2_transport.play(storage_options_, play_options_);

  await_received_messages.get();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::Primitives>(
    "/topic1");
  EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));
  EXPECT_THAT(replayed_test_primitives[0]->string_value, Eq(primitive_message1->string_value));
  EXPECT_THAT(replayed_test_primitives[1]->string_value, Eq(primitive_message2->string_value));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::StaticArrayPrimitives>(
    "/topic2");
  EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
  EXPECT_THAT(replayed_test_arrays[0]->bool_values, Eq(complex_message1->bool_values));
  EXPECT_THAT(replayed_test_arrays[0]->string_values, Eq(complex_message1->string_values));
  EXPECT_THAT(replayed_test_arrays[1]->bool_values, Eq(complex_message2->bool_values));
  EXPECT_THAT(replayed_test_arrays[1]->string_values, Eq(complex_message2->string_values));
}
