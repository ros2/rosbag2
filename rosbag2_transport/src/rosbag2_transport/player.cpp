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

#include "rcl/graph.h"

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"

#include "rosbag2_storage/storage_filter.hpp"

#include "rosbag2_transport/logging.hpp"

#include "qos.hpp"
#include "rosbag2_node.hpp"
#include "replayable_message.hpp"

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

Player::Player(
  std::shared_ptr<rosbag2_cpp::Reader> reader, std::shared_ptr<Rosbag2Node> rosbag2_transport)
: reader_(std::move(reader)), played_all_(false),
  rosbag2_transport_(rosbag2_transport), terminal_modified_(false)
{}

Player::~Player()
{
  restore_terminal();
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

  setup_terminal();
  std::future<void> keypress_future = std::async(
    std::launch::async,
    [this]() {handle_keypress();});

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

void Player::play_messages_from_queue(const PlayOptions & options)
{
  start_time_ = std::chrono::system_clock::now();
  paused_duration_ = std::chrono::nanoseconds(0);
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
  ReplayableMessage message;

  float rate = 1.0;
  // Use rate if in valid range
  if (options.rate > 0.0) {
    rate = options.rate;
  }

  while (message_queue_.try_dequeue(message) && rclcpp::ok()) {
    std::this_thread::sleep_until(
      start_time_ + std::chrono::duration_cast<std::chrono::nanoseconds>(
        1.0 / rate * message.time_since_start + paused_duration_));

    if (rosbag2_transport_->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      ROSBAG2_TRANSPORT_LOG_INFO("Paused");
      std::chrono::time_point<std::chrono::system_clock> pause_start =
        std::chrono::system_clock::now();

      // Sleep until activated externally
      while (rosbag2_transport_->get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE && rclcpp::ok())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
      }

      paused_duration_ += std::chrono::system_clock::now() - pause_start;

      if (rclcpp::ok()) {
        ROSBAG2_TRANSPORT_LOG_INFO("Resumed");
      }
    }

    if (rclcpp::ok()) {
      publishers_[message.message->topic_name]->publish(message.message->serialized_data);
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

void Player::handle_keypress()
{
  // Spin keyboard thread until all messages have been played
  while (rclcpp::ok() && !played_all_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    int input = read_char_from_stdin();
    switch (input) {
      case ' ':
        if (rosbag2_transport_->get_current_state().id() !=
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
          rosbag2_transport_->activate();
        } else {
          rosbag2_transport_->deactivate();
        }
        break;
    }
  }
}

void Player::setup_terminal()
{
  if (terminal_modified_) {
    return;
  }

#if defined(_MSC_VER)
  input_handle = GetStdHandle(STD_INPUT_HANDLE);
  if (input_handle == INVALID_HANDLE_VALUE) {
    std::cout << "Failed to set up standard input handle." << std::endl;
    return;
  }
  if (!GetConsoleMode(input_handle, &stdin_set)) {
    std::cout << "Failed to save the console mode." << std::endl;
    return;
  }
  terminal_modified_ = true;
#else
  const int fd = fileno(stdin);
  termios flags;
  tcgetattr(fd, &orig_flags_);
  flags = orig_flags_;
  flags.c_lflag &= ~ICANON;      // set raw (unset canonical modes)
  flags.c_cc[VMIN] = 0;         // i.e. min 1 char for blocking, 0 chars for non-blocking
  flags.c_cc[VTIME] = 0;         // block if waiting for char
  tcsetattr(fd, TCSANOW, &flags);

  FD_ZERO(&stdin_fdset_);
  FD_SET(fd, &stdin_fdset_);
  maxfd_ = fd + 1;
  terminal_modified_ = true;
#endif
}

void Player::restore_terminal()
{
  if (!terminal_modified_) {
    return;
  }

#if defined(_MSC_VER)
  SetConsoleMode(input_handle, stdin_set);
#else
  const int fd = fileno(stdin);
  tcsetattr(fd, TCSANOW, &orig_flags_);
#endif
  terminal_modified_ = false;
}

int Player::read_char_from_stdin()
{
#ifdef __APPLE__
  fd_set testfd;
  FD_COPY(&stdin_fdset_, &testfd);
#elif !defined(_MSC_VER)
  fd_set testfd = stdin_fdset_;
#endif

#if defined(_MSC_VER)
  DWORD events = 0;
  INPUT_RECORD input_record[1];
  DWORD input_size = 1;
  BOOL b = GetNumberOfConsoleInputEvents(input_handle, &events);
  if (b && events > 0) {
    b = ReadConsoleInput(input_handle, input_record, input_size, &events);
    if (b) {
      for (unsigned int i = 0; i < events; ++i) {
        if (input_record[i].EventType & KEY_EVENT & input_record[i].Event.KeyEvent.bKeyDown) {
          CHAR ch = input_record[i].Event.KeyEvent.uChar.AsciiChar;
          return ch;
        }
      }
    }
  }
  return EOF;
#else
  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  if (select(maxfd_, &testfd, NULL, NULL, &tv) <= 0) {
    return EOF;
  }
  return getc(stdin);
#endif
}

}  // namespace rosbag2_transport
