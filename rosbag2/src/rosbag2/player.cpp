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

#include "player.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl/graph.h"
#include "rcutils/time.h"

#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2/logging.hpp"
#include "rosbag2_node.hpp"
#include "replayable_message.hpp"
#include "typesupport_helpers.hpp"

namespace rosbag2
{

Player::Player(std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage)
: storage_(storage), node_(std::make_shared<Rosbag2Node>("rosbag2_node"))
{}

void Player::play()
{
  prepare_publishers();

  auto db_read_future = std::async(std::launch::async, [this]() {load_storage_content();});
  db_read_future.get();

  play_messages_from_queue();
}

void Player::load_storage_content()
{
  using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
  TimePoint time_first_message;

  ReplayableMessage message;
  if (storage_->has_next()) {
    message.message = storage_->read_next();
    message.time_since_start = std::chrono::nanoseconds(0);
    time_first_message = TimePoint(std::chrono::nanoseconds(message.message->time_stamp));
    message_queue_.push(message);
  }
  while (storage_->has_next()) {
    message.message = storage_->read_next();
    message.time_since_start =
      TimePoint(std::chrono::nanoseconds(message.message->time_stamp)) - time_first_message;

    message_queue_.push(message);
  }
}

void Player::play_messages_from_queue()
{
  auto start_time = std::chrono::high_resolution_clock::now();

  // TODO(holznera): only stop if also the storage has been read completly.
  while (!message_queue_.empty()) {
    const auto & message = message_queue_.front();
    std::this_thread::sleep_until(start_time + message.time_since_start);
    publishers_[message.message->topic_name]->publish(message.message->serialized_data);
    message_queue_.pop();
  }
}

void Player::prepare_publishers()
{
  auto all_topics_and_types = storage_->get_all_topics_and_types();
  for (const auto & element : all_topics_and_types) {
    publishers_.insert(std::make_pair(
        element.first, node_->create_generic_publisher(element.first, element.second)));
  }
}

}  // namespace rosbag2
