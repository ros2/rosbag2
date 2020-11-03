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
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

#include "rcl/graph.h"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"

#include "rosbag2_storage/storage_filter.hpp"

#include "rosbag2_transport/logging.hpp"

#include "qos.hpp"
#include "rosbag2_node.hpp"

namespace
{
/**
 * Determine which QoS to offer for a topic.
 * The priority of the profile selected is:
 *   1. The override specified in play_options (if one exists for the topic).
 *   2. A profile automatically adapted to the recorded QoS profiles of publishers on the topic.
 *
 * \param topic_name The full name of the topic, with namespace (ex. /arm/joint_status).
 * \param topic_qos_profile_overrides A map of topic to QoS profile overrides.
 * @return The QoS profile to be used for subscribing.
 */
rclcpp::QoS publisher_qos_for_topic(
  const rosbag2_storage::TopicMetadata & topic,
  const std::unordered_map<std::string, rclcpp::QoS> & topic_qos_profile_overrides)
{
  using rosbag2_transport::Rosbag2QoS;
  auto qos_it = topic_qos_profile_overrides.find(topic.name);
  if (qos_it != topic_qos_profile_overrides.end()) {
    ROSBAG2_TRANSPORT_LOG_INFO_STREAM("Overriding QoS profile for topic " << topic.name);
    return Rosbag2QoS{qos_it->second};
  } else if (topic.offered_qos_profiles.empty()) {
    return Rosbag2QoS{};
  }

  const auto profiles_yaml = YAML::Load(topic.offered_qos_profiles);
  const auto offered_qos_profiles = profiles_yaml.as<std::vector<Rosbag2QoS>>();
  return Rosbag2QoS::adapt_offer_to_recorded_offers(topic.name, offered_qos_profiles);
}
}  // namespace

namespace rosbag2_transport
{

const std::chrono::milliseconds
Player::queue_read_wait_period_ = std::chrono::milliseconds(100);
const std::chrono::milliseconds
Player::pause_sleep_period_ = std::chrono::milliseconds(30);

Player::Player(
  std::shared_ptr<rosbag2_cpp::Reader> reader, std::shared_ptr<Rosbag2Node> rosbag2_transport)
: reader_(std::move(reader)), played_all_(false),
  rosbag2_transport_(rosbag2_transport)
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

void Player::play(const PlayOptions & options)
{
  topic_qos_profile_overrides_ = options.topic_qos_profile_overrides;
  prepare_publishers(options);

  storage_loading_future_ = std::async(
    std::launch::async,
    [this, options]() {load_storage_content(options);});

  wait_for_filled_queue(options);

  rosbag2_transport_->configure();
  // If playback should not be paused in the beginning, activate lifecycle node right away.
  // If playback should be paused, move on.
  // Playback will detect when node is activated, either from here or externally.
  if (!options.paused) {
    rosbag2_transport_->activate();
  }

  play_messages_from_queue(options);
}

void Player::wait_for_filled_queue(const PlayOptions & options) const
{
  while (
    message_queue_.size_approx() < options.read_ahead_queue_size &&
    !is_storage_completely_loaded() && rclcpp::ok())
  {
    std::this_thread::sleep_for(queue_read_wait_period_);
  }
}

void Player::load_storage_content(const PlayOptions & options)
{

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;
  if (reader_->has_next()) {
    message = reader_->read_next();
    time_translator_.setRealStartTime(TimePoint(std::chrono::nanoseconds(message->time_stamp)));
    message_queue_.enqueue(message);
  }

  auto queue_lower_boundary =
    static_cast<size_t>(options.read_ahead_queue_size * read_ahead_lower_bound_percentage_);
  auto queue_upper_boundary = options.read_ahead_queue_size;

  while (reader_->has_next() && rclcpp::ok()) {
    if (message_queue_.size_approx() < queue_lower_boundary) {
      enqueue_up_to_boundary(queue_upper_boundary);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void Player::enqueue_up_to_boundary(uint64_t boundary)
{
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;
  for (size_t i = message_queue_.size_approx(); i < boundary; i++) {
    if (!reader_->has_next()) {
      break;
    }
    message = reader_->read_next();
    message_queue_.enqueue(message);
  }
}

void Player::play_messages_from_queue(const PlayOptions & options)
{
  time_translator_.setRealStartTime(std::chrono::system_clock::now());
  do {
    play_messages_until_queue_empty(options);
    if (!is_storage_completely_loaded() && rclcpp::ok()) {
      ROSBAG2_TRANSPORT_LOG_WARN(
        "Message queue starved. Messages will be delayed. Consider "
        "increasing the --read-ahead-queue-size option.");
    }
  } while (!is_storage_completely_loaded() && rclcpp::ok());

  played_all_ = true;
}

void Player::play_messages_until_queue_empty(const PlayOptions & options)
{
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;

  // Use rate if in valid range
  if (options.rate > 0.0) {
    time_translator_.setTimeScale(options.rate);
  }

  static size_t counter = 0;
  while (message_queue_.try_dequeue(message) && rclcpp::ok()) {
    // DJA: What happens if we sleep for a very long time, restart, and then
    // kill the system? Do we end up waiting here until the pause duration,
    // potentially making it look like the system is hanging during shutdown?
    std::this_thread::sleep_until(
      time_translator_.translate(std::chrono::nanoseconds(message->time_stamp)));

    if (rosbag2_transport_->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      ROSBAG2_TRANSPORT_LOG_INFO("Paused");
      time_translator_.startPause();

      // Sleep until activated externally
      while (rosbag2_transport_->get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE && rclcpp::ok())
      {
        std::this_thread::sleep_for(pause_sleep_period_);
      }

      time_translator_.endPause();

      if (rclcpp::ok()) {
        ROSBAG2_TRANSPORT_LOG_INFO("Resumed");
      }
    }

    if (rclcpp::ok()) {
      publishers_[message->topic_name]->publish(message->serialized_data);
      // TODO(mabelzhang) TEMPORARY, remove when done debugging. And counter above.
      fprintf(stderr, "publishing message %zu\n", (++counter));
    }
  }
}

void Player::prepare_publishers(const PlayOptions & options)
{
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = options.topics_to_filter;
  reader_->set_filter(storage_filter);

  auto topics = reader_->get_all_topics_and_types();
  for (const auto & topic : topics) {
    auto topic_qos = publisher_qos_for_topic(topic, topic_qos_profile_overrides_);
    publishers_.insert(
      std::make_pair(
        topic.name, rosbag2_transport_->create_generic_publisher(
          topic.name, topic.type, topic_qos)));
  }
}

}  // namespace rosbag2_transport
