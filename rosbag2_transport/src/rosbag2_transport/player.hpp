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

#ifndef ROSBAG2_TRANSPORT__PLAYER_HPP_
#define ROSBAG2_TRANSPORT__PLAYER_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>

#include "moodycamel/readerwriterqueue.h"

#include "rclcpp/qos.hpp"

#include "rosbag2_transport/play_options.hpp"

#include "replayable_message.hpp"

using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

namespace rosbag2_cpp
{
class Reader;
}  // namespace rosbag2_cpp

namespace rosbag2_transport
{

class GenericPublisher;
class Rosbag2Node;

class Player
{
public:
  explicit Player(
    std::shared_ptr<rosbag2_cpp::Reader> reader,
    std::shared_ptr<Rosbag2Node> rosbag2_transport);

  void play(const PlayOptions & options);

private:
  void load_storage_content(const PlayOptions & options);
  bool is_storage_completely_loaded() const;
  void enqueue_up_to_boundary(const TimePoint & time_first_message, uint64_t boundary);
  void wait_for_filled_queue(const PlayOptions & options) const;
  void play_messages_from_queue(const PlayOptions & options);
  void play_messages_until_queue_empty(const PlayOptions & options);
  void prepare_publishers();

  /**
   * Determine which QoS to offer for a topic.
   * The priority of the profile selected is:
   *   1. The override specified in play_options (if one exists for the topic).
   *   2. The default RMW QoS profile.
   *
   * \param topic_name The full name of the topic, with namespace (ex. /arm/joint_status).
   * @return The QoS profile to be used for subscribing.
   */
  rclcpp::QoS publisher_qos_for_topic(const std::string & topic_name);

  static constexpr double read_ahead_lower_bound_percentage_ = 0.9;
  static const std::chrono::milliseconds queue_read_wait_period_;

  std::shared_ptr<rosbag2_cpp::Reader> reader_;
  moodycamel::ReaderWriterQueue<ReplayableMessage> message_queue_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  mutable std::future<void> storage_loading_future_;
  std::shared_ptr<Rosbag2Node> rosbag2_transport_;
  std::unordered_map<std::string, std::shared_ptr<GenericPublisher>> publishers_;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_HPP_
