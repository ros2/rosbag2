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
#include <thread>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/subscription_manager.hpp"

#include "rosbag2_transport/rosbag2_transport.hpp"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_play_test_fixture.hpp"

TEST_F(RosBag2PlayTestFixture, messages_played_in_loop) {
  auto primitive_message1 = get_messages_basic_types()[0];
  const int test_value = 42;
  primitive_message1->int32_value = test_value;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"loop_test_topic", "test_msgs/BasicTypes", "", ""}
  };

  size_t num_messages = 3;
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages(num_messages,
    serialize_test_message("loop_test_topic", 700, primitive_message1));

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  size_t expected_number_of_messages = num_messages * 2;
  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    "/loop_test_topic",
    expected_number_of_messages);

  auto await_received_messages = sub_->spin_subscriptions();

  auto rosbag2_transport_ptr = std::make_shared<rosbag2_transport::Rosbag2Transport>(
    reader_,
    writer_,
    info_);
  bool loop_playback = true;
  std::thread loop_thread(&rosbag2_transport::Rosbag2Transport::play, rosbag2_transport_ptr,
    storage_options_,
    rosbag2_transport::PlayOptions{1000, "", 1.0, loop_playback});
  std::this_thread::sleep_for(std::chrono_literals::operator""s(1));
  rclcpp::shutdown();
  loop_thread.join();

  await_received_messages.get();
  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    "/loop_test_topic");

  EXPECT_THAT(replayed_test_primitives.size(), Ge(expected_number_of_messages));
  EXPECT_THAT(
    replayed_test_primitives,
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int32_value, test_value))));
}
