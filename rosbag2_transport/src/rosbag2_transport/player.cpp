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

#include "rosbag2_transport/player.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcl/graph.h"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"

#include "rosbag2_storage/storage_filter.hpp"

#include "rosbag2_transport/logging.hpp"

#include "qos.hpp"

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

const std::chrono::milliseconds Player::queue_read_wait_period_ = std::chrono::milliseconds(30);

Player::Player(
  std::shared_ptr<rosbag2_cpp::Reader> reader, std::shared_ptr<Rosbag2Node> rosbag2_transport)
: reader_(reader), rosbag2_transport_(rosbag2_transport)
{
  prev_msg_time_since_start_ = std::chrono::nanoseconds(0);
  total_time_in_pause_ = std::chrono::nanoseconds(0);
  playback_time_in_pause_ = std::chrono::nanoseconds(0);
}

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
  set_playback_rate(options.rate);
  topic_qos_profile_overrides_ = options.topic_qos_profile_overrides;
  prepare_publishers(options);

  storage_loading_future_ = std::async(
    std::launch::async,
    [this, options]() {load_storage_content(options);});

  wait_for_filled_queue(options);

  play_messages_from_queue();
  // Workaround! Sleep for some reasonable amount of time to make sure that publishers will not be
  // destroyed before they will sent last message on wire.
  std::this_thread::sleep_for(std::chrono::milliseconds(delay_after_playback_ms_));
}

void Player::set_playback_rate(float rate)
{
  // Use rate if in valid range
  if (rate > 0.0) {
    playback_rate_ = rate;
  }
}

float Player::get_playback_rate()
{
  return playback_rate_;
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
  TimePoint time_first_message;

  ReplayableMessage message;
  if (reader_->has_next()) {
    message.message = reader_->read_next();
    message.time_since_start = std::chrono::nanoseconds(0);
    time_first_message = TimePoint(std::chrono::nanoseconds(message.message->time_stamp));
    message_queue_.enqueue(message);
  }

  auto queue_lower_boundary =
    static_cast<size_t>(options.read_ahead_queue_size * read_ahead_lower_bound_percentage_);
  auto queue_upper_boundary = options.read_ahead_queue_size;

  while (reader_->has_next() && rclcpp::ok()) {
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
    if (!reader_->has_next()) {
      break;
    }
    message.message = reader_->read_next();
    message.time_since_start =
      TimePoint(std::chrono::nanoseconds(message.message->time_stamp)) - time_first_message;

    message_queue_.enqueue(message);
  }
}

void Player::pause_resume()
{
  std::lock_guard<std::mutex> lk(time_in_pause_mutex_);
  paused_ = !paused_;
  if (paused_) {
    pause_begin_time_ = std::chrono::steady_clock::now();
    ROSBAG2_TRANSPORT_LOG_INFO("Pause playing messages from bag..");
  } else {
    total_time_in_pause_ += std::chrono::steady_clock::now() - pause_begin_time_;
    ROSBAG2_TRANSPORT_LOG_INFO("Resume playing messages from bag..");
  }
}

bool Player::play_next()
{
  ReplayableMessage message;
  std::lock_guard<std::mutex> lk(time_in_pause_mutex_);
  if (paused_ && have_more_messages_to_play()) {
    ReplayableMessage message;
    if (message_queue_.try_dequeue(message) && rclcpp::ok()) {
      playback_time_in_pause_ += message.time_since_start - prev_msg_time_since_start_;
      play_message(message);
      return true;
    } else {
      if (rclcpp::ok()) {
        ROSBAG2_TRANSPORT_LOG_WARN("Failed to play next message. Message queue empty.");
      }
    }
  }
  return false;
}

void Player::play_message(const ReplayableMessage & message)
{
  if (rclcpp::ok()) {
    auto publisher_iter = publishers_.find(message.message->topic_name);
    if (publisher_iter != publishers_.end()) {
      publisher_iter->second->publish(message.message->serialized_data);
    }
    prev_msg_time_since_start_ = message.time_since_start;
  }
}

void Player::play_message_in_time(const ReplayableMessage & message)
{
  using namespace std::chrono;  // NOLINT
  std::this_thread::sleep_until(
    start_time_ + duration_cast<nanoseconds>(total_time_in_pause_) +
    duration_cast<nanoseconds>(
      1.0 / playback_rate_ * (message.time_since_start - playback_time_in_pause_)));
  play_message(message);
}

bool Player::have_more_messages_to_play()
{
  if (!is_storage_completely_loaded()) {
    return true;
  } else {
    if (message_queue_.peek() != nullptr) {
      return true;
    } else {
      return false;
    }
  }
}

void Player::play_messages_from_queue()
{
  start_time_ = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lk(time_in_pause_mutex_);
    pause_begin_time_ = start_time_;
    total_time_in_pause_ = std::chrono::nanoseconds(0);
  }
  prev_msg_time_since_start_ = std::chrono::nanoseconds(0);
  playback_time_in_pause_ = std::chrono::nanoseconds(0);
  ROSBAG2_TRANSPORT_LOG_INFO("Start playing messages from bag..");

  do {
    if (paused_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } else {
      play_messages_until_queue_empty();
      if (!paused_ && !is_storage_completely_loaded() && rclcpp::ok()) {
        ROSBAG2_TRANSPORT_LOG_WARN(
          "Message queue starved. Messages will be delayed. Consider "
          "increasing the --read-ahead-queue-size option.");
      }
    }
  } while (have_more_messages_to_play() && rclcpp::ok());

  if (!have_more_messages_to_play()) {
    ROSBAG2_TRANSPORT_LOG_INFO("Reached end of the bag");
  }
}

void Player::play_messages_until_queue_empty()
{
  ReplayableMessage message;
  while (!paused_ && message_queue_.try_dequeue(message) && rclcpp::ok()) {
    play_message_in_time(message);
  }
}

void Player::prepare_publishers(const PlayOptions & options)
{
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = options.topics_to_filter;
  reader_->set_filter(storage_filter);

  auto topics = reader_->get_all_topics_and_types();
  for (const auto & topic : topics) {
    // filter topics to add publishers if necessary
    auto & filter_topics = storage_filter.topics;
    if (!filter_topics.empty()) {
      auto iter = std::find(filter_topics.begin(), filter_topics.end(), topic.name);
      if (iter == filter_topics.end()) {
        continue;
      }
    }

    auto topic_qos = publisher_qos_for_topic(topic, topic_qos_profile_overrides_);
    try {
      publishers_.insert(
        std::make_pair(
          topic.name, rosbag2_transport_->create_generic_publisher(
            topic.name, topic.type, topic_qos)));
    } catch (const std::runtime_error & e) {
      // using a warning log seems better than adding a new option
      // to ignore some unknown message type library
      ROSBAG2_TRANSPORT_LOG_WARN(
        "Ignoring a topic '%s', reason: %s.", topic.name.c_str(), e.what());
    }
  }
}

}  // namespace rosbag2_transport
