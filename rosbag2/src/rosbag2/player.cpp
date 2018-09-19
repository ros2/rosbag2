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
#include <utility>

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

bool Player::is_storage_completely_loaded() const
{
  if (storage_loading_future_.valid() &&
    storage_loading_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    storage_loading_future_.get();
  }
  return !storage_loading_future_.valid();
}

void Player::play(const Rosbag2PlayOptions & options)
{
  prepare_publishers();

  storage_loading_future_ = std::async(std::launch::async,
      [this, options]() {load_storage_content(options);});

  wait_for_filled_queue(options);

  play_messages_from_queue();
}

void Player::wait_for_filled_queue(const Rosbag2PlayOptions & options) const
{
  while (
    message_queue_.size_approx() < options.read_ahead_queue_size &&
    !is_storage_completely_loaded())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void Player::load_storage_content(const Rosbag2PlayOptions & options)
{
  TimePoint time_first_message;

  ReplayableMessage message;
  if (storage_->has_next()) {
    message.message = storage_->read_next();
    message.time_since_start = std::chrono::nanoseconds(0);
    time_first_message = TimePoint(std::chrono::nanoseconds(message.message->time_stamp));
    message_queue_.enqueue(message);
  }

  auto queue_lower_boundary =
    static_cast<size_t>(options.read_ahead_queue_size * read_ahead_lower_bound_percentage_);
  auto queue_upper_boundary = options.read_ahead_queue_size;

  while (storage_->has_next()) {
    if (message_queue_.size_approx() < queue_lower_boundary) {
      enqueue_up_to_boundary(time_first_message, queue_upper_boundary);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void Player::enqueue_up_to_boundary(const TimePoint & time_first_message, uint64_t boundary)
{
  ReplayableMessage message;
  for (size_t i = message_queue_.size_approx(); i < boundary; i++) {
    if (!storage_->has_next()) {
      break;
    }
    message.message = storage_->read_next();
    message.time_since_start =
      TimePoint(std::chrono::nanoseconds(message.message->time_stamp)) - time_first_message;

    message_queue_.enqueue(message);
  }
}

void Player::play_messages_from_queue()
{
  auto start_time = std::chrono::high_resolution_clock::now();

  while (message_queue_.size_approx() != 0 || !is_storage_completely_loaded()) {
    ReplayableMessage message;
    if (message_queue_.try_dequeue(message)) {
      std::this_thread::sleep_until(start_time + message.time_since_start);
      publishers_[message.message->topic_name]->publish(message.message->serialized_data);
    }
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
