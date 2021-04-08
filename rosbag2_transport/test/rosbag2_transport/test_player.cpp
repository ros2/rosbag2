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
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_transport/player.hpp"

#include "mock_sequential_reader.hpp"
#include "rosbag2_cpp/reader.hpp"

using namespace ::testing;  // NOLINT

TEST(RosBag2TestPlayer, initialize_player_unique_ptr)
{
  rclcpp::init(0, nullptr);

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  rosbag2_storage::StorageOptions storage_options{};
  rosbag2_transport::PlayOptions play_options{};
  rosbag2_transport::Player player(std::move(reader), storage_options, play_options);

  rclcpp::shutdown();
}

TEST(RosBag2TestPlayer, initialize_player_move_constructor)
{
  rclcpp::init(0, nullptr);

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  rosbag2_cpp::Reader reader(std::move(prepared_mock_reader));
  rosbag2_storage::StorageOptions storage_options{};
  rosbag2_transport::PlayOptions play_options{};
  rosbag2_transport::Player player(std::move(reader), storage_options, play_options);
  player.play();

  rclcpp::shutdown();
}
