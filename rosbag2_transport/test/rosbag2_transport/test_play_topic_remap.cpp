// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2020, TNG Technology Consulting GmbH.
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
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_test_common/subscription_manager.hpp"

#include "rosbag2_transport/rosbag2_transport.hpp"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_play_test_fixture.hpp"

TEST_F(RosBag2PlayTestFixture, recorded_message_is_played_on_remapped_topic) {
  //  test constants
  const std::string original_topic = "/topic_before_remap";
  const std::string remapped_topic = "/topic_after_remap";
  const int32_t test_value = 42;
  const int64_t dt_in_milliseconds = 100;
  const size_t expected_number_of_outgoing_messages = 5;
  play_options_.topic_remapping_options =
  {"--ros-args", "--remap", "topic_before_remap:=topic_after_remap"};

  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = test_value;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {original_topic, "test_msgs/BasicTypes", "", ""}
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
  for (auto i = 1u; i <= expected_number_of_outgoing_messages; ++i) {
    messages.push_back(
      serialize_test_message(original_topic, dt_in_milliseconds * i, primitive_message1));
  }
  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  // We can't reliably test for an exact number of messages to arrive.
  // Therefore, we test that at least one message is being received over the remapped topic.
  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    remapped_topic, 1u);
  auto await_received_messages = sub_->spin_subscriptions();

  rosbag2_transport::Rosbag2Transport rosbag2_transport(reader_, writer_, info_);

  rosbag2_transport.play(storage_options_, play_options_);

  await_received_messages.get();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    remapped_topic);
  EXPECT_FALSE(replayed_test_primitives.empty());
  EXPECT_THAT(
    replayed_test_primitives,
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int32_value, test_value))));
}
