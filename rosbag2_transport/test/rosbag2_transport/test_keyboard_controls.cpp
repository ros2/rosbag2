// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <future>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/subscription_manager.hpp"

#include "rosbag2_transport/player.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_transport_test_fixture.hpp"
#include "mock_keyboard_handler.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

// wrapper player that counts command calls
class CountPlayer : public Player
{
public:
  CountPlayer(
    std::unique_ptr<rosbag2_cpp::Reader> reader,
    std::shared_ptr<KeyboardHandler> keyboard_handler,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options)
  : Player(std::move(reader), keyboard_handler, storage_options, play_options) {}

  void pause() override
  {
    Player::pause();
    num_paused++;
  }

  void resume() override
  {
    Player::resume();
    num_resumed++;
  }

  bool play_next() override
  {
    bool ret = Player::play_next();
    if (ret) {
      num_played_next++;
    }
    return ret;
  }

  int num_paused = 0;
  int num_resumed = 0;
  int num_played_next = 0;
};


TEST_F(RosBag2PlayTestFixture, invalid_keybindings)
{
  auto primitive_message1 = get_messages_strings()[0];
  primitive_message1->string_value = "Hello World 1";

  auto primitive_message2 = get_messages_strings()[0];
  primitive_message2->string_value = "Hello World 2";

  auto message_time_difference = rclcpp::Duration(1, 0);
  auto topics_and_types =
    std::vector<rosbag2_storage::TopicMetadata>{{"topic1", "test_msgs/Strings", "", ""}};
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 0, primitive_message1),
    serialize_test_message("topic1", 0, primitive_message2)};

  messages[0]->time_stamp = 100;
  messages[1]->time_stamp = messages[0]->time_stamp + message_time_difference.nanoseconds();

  play_options_.play_next_key = KeyboardHandler::KeyCode::UNKNOWN;

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topics_and_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  std::shared_ptr<Player> player;
  EXPECT_THROW(
    player = std::make_shared<Player>(std::move(reader), storage_options_, play_options_),
    std::invalid_argument
  );
}

TEST_F(RosBag2PlayTestFixture, test_keyboard_controls)
{
  auto primitive_message1 = get_messages_strings()[0];
  primitive_message1->string_value = "Hello World 1";

  auto primitive_message2 = get_messages_strings()[0];
  primitive_message2->string_value = "Hello World 2";

  auto primitive_message3 = get_messages_strings()[0];
  primitive_message3->string_value = "Hello World 3";

  auto message_time_difference = rclcpp::Duration(1, 0);
  auto topics_and_types =
    std::vector<rosbag2_storage::TopicMetadata>{{"topic1", "test_msgs/Strings", "", ""}};
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 0, primitive_message1),
    serialize_test_message("topic1", 0, primitive_message2),
    serialize_test_message("topic1", 0, primitive_message3)};

  messages[0]->time_stamp = 100;
  messages[1]->time_stamp = messages[0]->time_stamp + message_time_difference.nanoseconds();
  messages[2]->time_stamp = messages[1]->time_stamp + message_time_difference.nanoseconds();

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topics_and_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto keyboard_handler = std::make_shared<MockKeyboardHandler>();

  // play bag in thread
  auto player = std::make_shared<CountPlayer>(
    std::move(reader), keyboard_handler,
    storage_options_, play_options_);

  // start in pause mode
  EXPECT_THAT(player->is_paused(), false);
  keyboard_handler->simulate_key_press(play_options_.pause_resume_toggle_key);
  EXPECT_THAT(player->is_paused(), true);

  // start play thread
  std::thread player_thread = std::thread([player]() {player->play();});

  // play next
  keyboard_handler->simulate_key_press(play_options_.play_next_key);
  EXPECT_THAT(player->is_paused(), true);
  // resume
  keyboard_handler->simulate_key_press(play_options_.pause_resume_toggle_key);
  EXPECT_THAT(player->is_paused(), false);

  if (player_thread.joinable()) {
    player_thread.join();
  }

  EXPECT_THAT(player->num_paused, 1);
  EXPECT_THAT(player->num_resumed, 1);
  EXPECT_THAT(player->num_played_next, 1);
}
