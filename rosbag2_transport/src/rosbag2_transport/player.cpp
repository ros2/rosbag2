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

Player::Player(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  // TODO(karsten1987): Use this constructor later with parameter parsing.
  // The reader, storage_options as well as play_options can be loaded via parameter.
  // That way, the player can be used as a simple component in a component manager.
  throw rclcpp::exceptions::UnimplementedError();
}

Player::Player(
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::PlayOptions & play_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Player(std::make_unique<rosbag2_cpp::Reader>(),
    storage_options, play_options,
    node_name, node_options)
{}

Player::Player(
  std::unique_ptr<rosbag2_cpp::Reader> reader,
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
  {
    reader_->open(storage_options_, {"", rmw_get_serialization_format()});
    const auto starting_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
      reader_->get_metadata().starting_time.time_since_epoch()).count();
    clock_ = std::make_unique<rosbag2_cpp::TimeControllerClock>(starting_time);
    set_rate(play_options_.rate);

    topic_qos_profile_overrides_ = play_options_.topic_qos_profile_overrides;
    prepare_publishers();

    reader_->close();
  }

  srv_pause_ = create_service<rosbag2_interfaces::srv::Pause>(
    "~/pause",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::Pause::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::Pause::Response>/* response */)
    {
      pause();
    });
  srv_resume_ = create_service<rosbag2_interfaces::srv::Resume>(
    "~/resume",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::Resume::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::Resume::Response>/* response */)
    {
      resume();
    });
  srv_toggle_paused_ = create_service<rosbag2_interfaces::srv::TogglePaused>(
    "~/toggle_paused",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::TogglePaused::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::TogglePaused::Response>/* response */)
    {
      toggle_paused();
    });
  srv_is_paused_ = create_service<rosbag2_interfaces::srv::IsPaused>(
    "~/is_paused",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::IsPaused::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::IsPaused::Response> response)
    {
      response->paused = is_paused();
    });
  srv_get_rate_ = create_service<rosbag2_interfaces::srv::GetRate>(
    "~/get_rate",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::GetRate::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::GetRate::Response> response)
    {
      response->rate = get_rate();
    });
  srv_set_rate_ = create_service<rosbag2_interfaces::srv::SetRate>(
    "~/set_rate",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::SetRate::Request> request,
      const std::shared_ptr<rosbag2_interfaces::srv::SetRate::Response> response)
    {
      response->success = set_rate(request->rate);
    });
  srv_play_next_ = create_service<rosbag2_interfaces::srv::PlayNext>(
    "~/play_next",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::PlayNext::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::PlayNext::Response> response)
    {
      response->success = play_next();
    });
}

Player::~Player()
{
  if (reader_) {
    reader_->close();
  }
}

rosbag2_cpp::Reader * Player::release_reader()
{
  reader_->close();
  return reader_.release();
}

const std::chrono::milliseconds
Player::queue_read_wait_period_ = std::chrono::milliseconds(100);

bool Player::is_storage_completely_loaded() const
{
  if (storage_loading_future_.valid() &&
    storage_loading_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    storage_loading_future_.get();
  }
  return !storage_loading_future_.valid();
}

void Player::play()
{
  is_in_play_ = true;

  float delay;
  if (play_options_.delay > 0.0){
    delay = play_options_.delay;
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Invalid delay value: %f. Delay is disabled.",
                play_options_.delay);

    delay = 0.0;
  }

  try {
    do {
      if (delay > 0.0) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Sleep " << delay << " sec");
        std::chrono::duration<float> duration(delay);
        std::this_thread::sleep_for(duration);
      }
      reader_->open(storage_options_, {"", rmw_get_serialization_format()});
      const auto starting_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
        reader_->get_metadata().starting_time.time_since_epoch()).count();
      clock_->jump(starting_time);

      storage_loading_future_ = std::async(
        std::launch::async,
        [this]() {load_storage_content();});

      wait_for_filled_queue();
      play_messages_from_queue();
      reader_->close();
    } while (rclcpp::ok() && play_options_.loop);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to play: %s", e.what());
  }
  is_in_play_ = false;
}

void Player::pause()
{
  clock_->pause();
}

void Player::resume()
{
  clock_->resume();
}

void Player::toggle_paused()
{
  clock_->is_paused() ? clock_->resume() : clock_->pause();
}

bool Player::is_paused() const
{
  return clock_->is_paused();
}

double Player::get_rate() const
{
  return clock_->get_rate();
}

bool Player::set_rate(double rate)
{
  return clock_->set_rate(rate);
}

rosbag2_storage::SerializedBagMessageSharedPtr * Player::peek_next_message_from_queue()
{
  rosbag2_storage::SerializedBagMessageSharedPtr * message_ptr = message_queue_.peek();
  if (message_ptr == nullptr && !is_storage_completely_loaded() && rclcpp::ok()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Message queue starved. Messages will be delayed. Consider "
      "increasing the --read-ahead-queue-size option.");
    while (message_ptr == nullptr && !is_storage_completely_loaded() && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      message_ptr = message_queue_.peek();
    }
  }
  return message_ptr;
}

bool Player::play_next()
{
  // Temporary take over playback from play_messages_from_queue()
  std::lock_guard<std::mutex> lk(skip_message_in_main_play_loop_mutex_);

  if (!clock_->is_paused() || !is_in_play_) {
    return false;
  }

  skip_message_in_main_play_loop_ = true;
  rosbag2_storage::SerializedBagMessageSharedPtr * message_ptr = peek_next_message_from_queue();

  bool next_message_published = false;
  while (message_ptr != nullptr && !next_message_published) {
    {
      rosbag2_storage::SerializedBagMessageSharedPtr message = *message_ptr;
      next_message_published = publish_message(message);
      clock_->jump(message->time_stamp);
    }
    message_queue_.pop();
    message_ptr = peek_next_message_from_queue();
  }
  return next_message_published;
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
  rosbag2_storage::SerializedBagMessageSharedPtr message;
  for (size_t i = message_queue_.size_approx(); i < boundary; i++) {
    if (!reader_->has_next()) {
      break;
    }
    message = reader_->read_next();
    message_queue_.enqueue(message);
  }
}

void Player::play_messages_from_queue()
{
  playing_messages_from_queue_ = true;
  // Note: We need to use message_queue_.peek() instead of message_queue_.try_dequeue(message)
  // to support play_next() API logic.
  rosbag2_storage::SerializedBagMessageSharedPtr * message_ptr = peek_next_message_from_queue();
  while (message_ptr != nullptr && rclcpp::ok()) {
    {
      rosbag2_storage::SerializedBagMessageSharedPtr message = *message_ptr;
      // Do not move on until sleep_until returns true
      // It will always sleep, so this is not a tight busy loop on pause
      while (rclcpp::ok() && !clock_->sleep_until(message->time_stamp)) {}
      if (rclcpp::ok()) {
        {
          std::lock_guard<std::mutex> lk(skip_message_in_main_play_loop_mutex_);
          if (skip_message_in_main_play_loop_) {
            skip_message_in_main_play_loop_ = false;
            message_ptr = peek_next_message_from_queue();
            continue;
          }
        }
        publish_message(message);
      }
      message_queue_.pop();
      message_ptr = peek_next_message_from_queue();
    }
  }
  playing_messages_from_queue_ = false;
}

void Player::prepare_publishers()
{
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = play_options_.topics_to_filter;
  reader_->set_filter(storage_filter);

  // Create /clock publisher
  if (play_options_.clock_publish_frequency > 0.f) {
    const auto publish_period = std::chrono::nanoseconds(
      static_cast<uint64_t>(RCUTILS_S_TO_NS(1) / play_options_.clock_publish_frequency));
    // NOTE: PlayerClock does not own this publisher because rosbag2_cpp
    // should not own transport-based functionality
    clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS());
    clock_publish_timer_ = this->create_wall_timer(
      publish_period, [this]() {
        auto msg = rosgraph_msgs::msg::Clock();
        msg.clock = rclcpp::Time(clock_->now());
        clock_publisher_->publish(msg);
      });
  }

  // Create topic publishers
  auto topics = reader_->get_all_topics_and_types();
  for (const auto & topic : topics) {
    if (publishers_.find(topic.name) != publishers_.end()) {
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
      topic, topic_qos_profile_overrides_,
      this->get_logger());
    try {
      publishers_.insert(
        std::make_pair(
          topic.name, this->create_generic_publisher(topic.name, topic.type, topic_qos)));
    } catch (const std::runtime_error & e) {
      // using a warning log seems better than adding a new option
      // to ignore some unknown message type library
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring a topic '%s', reason: %s.", topic.name.c_str(), e.what());
    }
  }
}

bool Player::publish_message(rosbag2_storage::SerializedBagMessageSharedPtr message)
{
  bool message_published = false;
  auto publisher_iter = publishers_.find(message->topic_name);
  if (publisher_iter != publishers_.end()) {
    publisher_iter->second->publish(rclcpp::SerializedMessage(*message->serialized_data));
    message_published = true;
  }
  return message_published;
}

}  // namespace rosbag2_transport
