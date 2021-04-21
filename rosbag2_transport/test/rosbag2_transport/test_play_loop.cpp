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

#include "rosbag2_transport/player.hpp"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_play_test_fixture.hpp"

TEST_F(RosBag2PlayTestFixture, play_bag_file_twice) {
  //  test constants
  const int test_value = 42;
  const size_t num_messages = 3;
  const int64_t time_stamp_in_milliseconds = 700;
  const size_t expected_number_of_messages = num_messages * 2;
  const size_t read_ahead_queue_size = 1000;
  const float rate = 1.0;
  const bool loop_playback = false;

  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = test_value;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"loop_test_topic", "test_msgs/BasicTypes", "", ""}
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages(num_messages,
    serialize_test_message("loop_test_topic", time_stamp_in_milliseconds, primitive_message1));

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    "/loop_test_topic",
    expected_number_of_messages);

  auto await_received_messages = sub_->spin_subscriptions();

  rosbag2_transport::PlayOptions play_options =
  {read_ahead_queue_size, "", rate, {}, {}, loop_playback, {}};
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(
      reader), storage_options_, play_options);

  auto loop_thread = std::async(
    std::launch::async, [&player]() {
      player->play();
      // play again the same bag file
      player->play();
    });

  await_received_messages.get();
  rclcpp::shutdown();
  loop_thread.get();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    "/loop_test_topic");

  // TODO(karsten1987): FastRTPS loses repeatedly at least one message
  EXPECT_THAT(replayed_test_primitives.size(), Ge(expected_number_of_messages - 1));
  EXPECT_THAT(
    replayed_test_primitives,
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int32_value, test_value))));
}

TEST_F(RosBag2PlayTestFixture, messages_played_in_loop) {
  //  test constants
  const int test_value = 42;
  const size_t num_messages = 3;
  const int64_t time_stamp_in_milliseconds = 700;
  const size_t expected_number_of_messages = num_messages * 3;
  const size_t read_ahead_queue_size = 1000;
  const float rate = 1.0;
  const bool loop_playback = true;

  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = test_value;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"loop_test_topic", "test_msgs/BasicTypes", "", ""}
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages(num_messages,
    serialize_test_message("loop_test_topic", time_stamp_in_milliseconds, primitive_message1));

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    "/loop_test_topic",
    expected_number_of_messages);

  auto await_received_messages = sub_->spin_subscriptions();

  rosbag2_transport::PlayOptions play_options{read_ahead_queue_size, "", rate, {}, {},
    loop_playback, {}};
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(
      reader), storage_options_, play_options);
  std::thread loop_thread(&rosbag2_transport::Player::play, player);

  await_received_messages.get();
  rclcpp::shutdown();
  loop_thread.join();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    "/loop_test_topic");

  EXPECT_THAT(replayed_test_primitives.size(), Ge(expected_number_of_messages));
  EXPECT_THAT(
    replayed_test_primitives,
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int32_value, test_value))));
}
