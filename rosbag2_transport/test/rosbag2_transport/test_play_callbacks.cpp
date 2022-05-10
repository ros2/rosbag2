// Copyright 2020-2021, Apex.AI.
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
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "mock_player.hpp"
#include "rosbag2_transport_test_fixture.hpp"
#include "rosbag2_transport/player.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class Rosbag2PlayCallbacksTestFixture : public Rosbag2TransportTestFixture
{
public:
  Rosbag2PlayCallbacksTestFixture()
  : Rosbag2TransportTestFixture()
  {
    rclcpp::init(0, nullptr);

    auto topics_and_types =
      std::vector<rosbag2_storage::TopicMetadata>{{"topic1", "test_msgs/Strings", "", ""}};

    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
    messages.reserve(num_test_messages_);

    auto primitive_message = get_messages_strings()[0];
    primitive_message->string_value = "Hello World";

    rcutils_time_point_value_t current_timestamp =
      std::chrono::steady_clock::now().time_since_epoch().count();
    for (size_t i = 0; i < num_test_messages_; i++) {
      auto serialized_test_message =
        serialize_test_message("topic1", RCUTILS_NS_TO_MS(current_timestamp), primitive_message);
      messages.push_back(serialized_test_message);
      current_timestamp += message_time_difference_.count();
    }

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topics_and_types);
    reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  }

  ~Rosbag2PlayCallbacksTestFixture() override
  {
    rclcpp::shutdown();
  }

  const size_t num_test_messages_ = 3;
  const std::chrono::nanoseconds message_time_difference_ = 5ms;
};

TEST_F(Rosbag2PlayCallbacksTestFixture, nullptr_as_callback) {
  MockPlayer player(move(reader_), storage_options_, play_options_);
  EXPECT_EQ(player.add_on_play_message_pre_callback(nullptr), Player::invalid_callback_handle);
  EXPECT_EQ(player.add_on_play_message_post_callback(nullptr), Player::invalid_callback_handle);

  EXPECT_EQ(player.get_number_of_registered_pre_callbacks(), 0U);
  EXPECT_EQ(player.get_number_of_registered_post_callbacks(), 0U);
}

TEST_F(Rosbag2PlayCallbacksTestFixture, register_unregister_callbacks) {
  MockPlayer player(move(reader_), storage_options_, play_options_);

  auto lambda_as_callback = [](std::shared_ptr<rosbag2_storage::SerializedBagMessage>) {
      ASSERT_FALSE(true) << "This code should not be called \n";
    };

  auto pre_callback_handle = player.add_on_play_message_pre_callback(lambda_as_callback);
  ASSERT_NE(pre_callback_handle, Player::invalid_callback_handle);
  ASSERT_EQ(player.get_number_of_registered_pre_callbacks(), 1U);

  auto post_callback_handle = player.add_on_play_message_post_callback(lambda_as_callback);
  ASSERT_NE(post_callback_handle, Player::invalid_callback_handle);
  ASSERT_EQ(player.get_number_of_registered_post_callbacks(), 1U);

  // Check for correctness of call delete_on_play_message_callback with invalid_callback_handle
  player.delete_on_play_message_callback(Player::invalid_callback_handle);
  ASSERT_EQ(player.get_number_of_registered_pre_callbacks(), 1U);
  ASSERT_EQ(player.get_number_of_registered_post_callbacks(), 1U);

  player.delete_on_play_message_callback(pre_callback_handle);
  ASSERT_EQ(player.get_number_of_registered_pre_callbacks(), 0U);
  ASSERT_EQ(player.get_number_of_registered_post_callbacks(), 1U);

  player.delete_on_play_message_callback(post_callback_handle);
  ASSERT_EQ(player.get_number_of_registered_pre_callbacks(), 0U);
  ASSERT_EQ(player.get_number_of_registered_post_callbacks(), 0U);

  player.pause();   // Put player in pause mode before starting
  ASSERT_TRUE(player.is_paused());

  // Run play asynchronously in separate thread
  std::future<void> play_future_result =
    std::async(std::launch::async, [&]() {player.play();});

  for (size_t i = 1; i < num_test_messages_; i++) {
    EXPECT_TRUE(player.play_next());
  }
  player.resume();   // Resume playback for playing the last message
  ASSERT_FALSE(player.is_paused());

  play_future_result.wait();
  play_future_result.get();
  EXPECT_FALSE(player.play_next());
}

TEST_F(Rosbag2PlayCallbacksTestFixture, call_callbacks) {
  using SerializedBagMessage = rosbag2_storage::SerializedBagMessage;
  MockPlayer player(move(reader_), storage_options_, play_options_);

  testing::MockFunction<void(std::shared_ptr<SerializedBagMessage>)> mock_pre_callback;
  EXPECT_CALL(mock_pre_callback, Call(_)).Times(Exactly(num_test_messages_));

  testing::MockFunction<void(std::shared_ptr<SerializedBagMessage>)> mock_post_callback;
  EXPECT_CALL(mock_post_callback, Call(_)).Times(Exactly(num_test_messages_));

  auto pre_callback_handle =
    player.add_on_play_message_pre_callback(mock_pre_callback.AsStdFunction());
  ASSERT_NE(pre_callback_handle, Player::invalid_callback_handle);
  ASSERT_EQ(player.get_number_of_registered_pre_callbacks(), 1U);

  auto post_callback_handle =
    player.add_on_play_message_post_callback(mock_post_callback.AsStdFunction());
  ASSERT_NE(post_callback_handle, Player::invalid_callback_handle);
  ASSERT_EQ(player.get_number_of_registered_post_callbacks(), 1U);

  player.pause();  // Put player in pause mode before starting
  ASSERT_TRUE(player.is_paused());

  // Run play asynchronously in separate thread
  std::future<void> play_future_result =
    std::async(std::launch::async, [&]() {player.play();});

  for (size_t i = 1; i < num_test_messages_; i++) {
    EXPECT_TRUE(player.play_next());
  }

  player.resume();  // Resume playback for playing the last message
  ASSERT_FALSE(player.is_paused());

  play_future_result.wait();
  play_future_result.get();
  EXPECT_FALSE(player.play_next());
}
