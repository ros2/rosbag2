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
#include <atomic>
#include <queue>
#include <string>
#include <unordered_map>

#include "moodycamel/readerwriterqueue.h"

#include "rclcpp/qos.hpp"
#include "rosbag2_transport/play_options.hpp"
#include "replayable_message.hpp"
#include "rosbag2_node.hpp"
#include "rosbag2_transport/visibility_control.hpp"

using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

namespace rosbag2_cpp
{
class Reader;
}  // namespace rosbag2_cpp

namespace rosbag2_transport
{

class GenericPublisher;

class Player
{
public:
  ROSBAG2_TRANSPORT_PUBLIC
  explicit Player(
    std::shared_ptr<rosbag2_cpp::Reader> reader,
    std::shared_ptr<Rosbag2Node> rosbag2_transport);

  ROSBAG2_TRANSPORT_PUBLIC
  void play(const PlayOptions & options);

  ROSBAG2_TRANSPORT_PUBLIC
  void set_playback_rate(float rate);

  ROSBAG2_TRANSPORT_PUBLIC
  float get_playback_rate();

  ROSBAG2_TRANSPORT_PUBLIC
  void pause_resume();

  ROSBAG2_TRANSPORT_PUBLIC
  bool play_next();

private:
  void load_storage_content(const PlayOptions & options);
  bool is_storage_completely_loaded() const;
  void enqueue_up_to_boundary(const TimePoint & time_first_message, uint64_t boundary);
  void wait_for_filled_queue(const PlayOptions & options) const;
  bool have_more_messages_to_play();
  void play_messages_from_queue();
  void play_messages_until_queue_empty();
  void prepare_publishers(const PlayOptions & options);
  void play_message(const ReplayableMessage & message);
  void play_message_in_time(const ReplayableMessage & message);

  static constexpr double read_ahead_lower_bound_percentage_ = 0.9;
  const size_t delay_after_playback_ms_ = 50;
  static const std::chrono::milliseconds queue_read_wait_period_;
  std::atomic<float> playback_rate_{1.0};
  bool paused_ = false;

  std::shared_ptr<rosbag2_cpp::Reader> reader_;
  moodycamel::ReaderWriterQueue<ReplayableMessage> message_queue_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  std::chrono::time_point<std::chrono::system_clock> pause_begin_time_;
  std::mutex time_in_pause_mutex_;
  std::chrono::nanoseconds prev_msg_time_since_start_;
  std::chrono::nanoseconds total_time_in_pause_;
  std::chrono::nanoseconds playback_time_in_pause_;
  mutable std::future<void> storage_loading_future_;
  std::shared_ptr<Rosbag2Node> rosbag2_transport_;
  std::unordered_map<std::string, std::shared_ptr<GenericPublisher>> publishers_;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_HPP_
