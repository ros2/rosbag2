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
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcl/graph.h"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/time.h"

#include "rosbag2_cpp/clocks/time_controller_clock.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"

#include "rosbag2_storage/storage_filter.hpp"

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
  const std::unordered_map<std::string, rclcpp::QoS> & topic_qos_profile_overrides,
  const rclcpp::Logger & logger)
{
  using rosbag2_transport::Rosbag2QoS;
  auto qos_it = topic_qos_profile_overrides.find(topic.name);
  if (qos_it != topic_qos_profile_overrides.end()) {
    RCLCPP_INFO_STREAM(
      logger,
      "Overriding QoS profile for topic " << topic.name);
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

Player::Player(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  // TODO(karsten1987): Use this constructor later with parameter parsing.
  // The reader, storage_options as well as play_options can be loaded via parameter.
  // That way, the player can be used as a simple component in a component manager.
  throw rclcpp::exceptions::UnimplementedError();
}

Player::Player(
  std::unique_ptr<rosbag2_cpp::Reader> reader,
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::PlayOptions & play_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Player(std::move(*reader.release()), storage_options, play_options, node_name, node_options)
{
}

Player::Player(
  rosbag2_cpp::Reader && reader,
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::PlayOptions & play_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(
    node_name,
    rclcpp::NodeOptions(node_options).arguments(play_options.topic_remapping_options)),
  reader_(std::move(reader)),
  storage_options_(storage_options),
  play_options_(play_options)
{
}

Player::~Player()
{
  reader_.reset();
}

void Player::play()
{
  do {
    play_once();
  } while (rclcpp::ok() && play_options_.loop);
}

void Player::play_once()
{
  reader_.open(storage_options_, {"", rmw_get_serialization_format()});
  if (reader_.has_next()) {
    // Reader does not have "peek", so we must "pop" the first message to see its timestamp
    auto message = reader_.read_next();
    prepare_clock(message->time_stamp);
    // Make sure that first message gets played by putting it into the play queue
    message_queue_.enqueue(message);
  } else {
    // The bag contains no messages - there is nothing to play
    reader_.reset();
    return;
  }

  topic_qos_profile_overrides_ = play_options_.topic_qos_profile_overrides;
  prepare_publishers();

  storage_loading_future_ = std::async(
    std::launch::async,
    [this]() {load_storage_content();});

  wait_for_filled_queue();

  play_messages_from_queue();

  reader_.reset();
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

void Player::wait_for_filled_queue() const
{
  while (
    message_queue_.size_approx() < play_options_.read_ahead_queue_size &&
    !is_storage_completely_loaded() && rclcpp::ok())
  {
    std::this_thread::sleep_for(queue_read_wait_period_);
  }
}

void Player::load_storage_content()
{
  auto queue_lower_boundary =
    static_cast<size_t>(play_options_.read_ahead_queue_size * read_ahead_lower_bound_percentage_);
  auto queue_upper_boundary = play_options_.read_ahead_queue_size;

  while (reader_.has_next() && rclcpp::ok()) {
    if (message_queue_.size_approx() < queue_lower_boundary) {
      enqueue_up_to_boundary(queue_upper_boundary);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void Player::enqueue_up_to_boundary(uint64_t boundary)
{
  rosbag2_storage::SerializedBagMessageSharedPtr message;
  for (size_t i = message_queue_.size_approx(); i < boundary; i++) {
    if (!reader_.has_next()) {
      break;
    }
    message = reader_.read_next();
    message_queue_.enqueue(message);
  }
}

void Player::play_messages_from_queue()
{
  do {
    play_messages_until_queue_empty();
    if (!is_storage_completely_loaded() && rclcpp::ok()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Message queue starved. Messages will be delayed. Consider "
        "increasing the --read-ahead-queue-size option.");
    }
  } while (!is_storage_completely_loaded() && rclcpp::ok());
}

void Player::play_messages_until_queue_empty()
{
  rosbag2_storage::SerializedBagMessageSharedPtr message;
  while (message_queue_.try_dequeue(message) && rclcpp::ok()) {
    // Do not move on until sleep_until returns true
    // It will always sleep, so this is not a tight busy loop on pause
    while (rclcpp::ok() && !clock_->sleep_until(message->time_stamp)) {}
    if (rclcpp::ok()) {
      auto publisher_iter = publishers_.find(message->topic_name);
      if (publisher_iter != publishers_.end()) {
        publisher_iter->second->publish(rclcpp::SerializedMessage(*message->serialized_data.get()));
      }
    }
  }
}

// TODO(karsten1987): This should be done in the constructor
void Player::prepare_publishers()
{
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = play_options_.topics_to_filter;
  reader_.set_filter(storage_filter);

  auto topics = reader_.get_all_topics_and_types();
  for (const auto & topic : topics) {
    // publisher is already prepared (e.g. in case of play loop)
    if (publishers_.find(topic.name) != publishers_.end())
    {
      continue;
    }

    // filter topics to add publishers if necessary
    auto & filter_topics = storage_filter.topics;
    if (!filter_topics.empty()) {
      auto iter = std::find(filter_topics.begin(), filter_topics.end(), topic.name);
      if (iter == filter_topics.end()) {
        continue;
      }
    }

    auto topic_qos = publisher_qos_for_topic(
      topic, topic_qos_profile_overrides_, this->get_logger());
    try {
      publishers_.insert(
        std::make_pair(
          topic.name,
          this->create_generic_publisher(topic.name, topic.type, topic_qos)));
    } catch (const std::runtime_error & e) {
      // using a warning log seems better than adding a new option
      // to ignore some unknown message type library
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring a topic '%s', reason: %s.", topic.name.c_str(), e.what());
    }
  }
}

void Player::prepare_clock(rcutils_time_point_value_t starting_time)
{
  double rate = play_options_.rate > 0.0 ? play_options_.rate : 1.0;
  clock_ = std::make_unique<rosbag2_cpp::TimeControllerClock>(starting_time, rate);
}

}  // namespace rosbag2_transport
