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

#ifndef ROSBAG2_TRANSPORT__PLAY_OPTIONS_HPP_
#define ROSBAG2_TRANSPORT__PLAY_OPTIONS_HPP_

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "keyboard_handler/keyboard_handler.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"

namespace rosbag2_transport
{

struct PlayOptions
{
public:
  size_t read_ahead_queue_size = 1000;
  std::string node_prefix = "";
  float rate = 1.0;

  // Topic names to whitelist when playing a bag.
  // Only messages matching these specified topics will be played.
  // If list is empty, the filter is ignored and all messages are played.
  std::vector<std::string> topics_to_filter = {};

  // Regular expressions of topic names to whitelist when playing a bag.
  // Only messages matching these specified topics will be played.
  // If list is empty, the filter is ignored and all messages are played.
  std::vector<std::string> regex_to_filter = {};

  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides = {};
  bool loop = false;
  std::vector<std::string> topic_remapping_options = {};

  // Rate in Hz at which to publish to /clock.
  // 0 (or negative) means that no publisher will be created
  double clock_publish_frequency = 0.0;

  // Sleep before play. Negative durations invalid. Will delay at the beginning of each loop.
  rclcpp::Duration delay = rclcpp::Duration(0, 0);

  // Start paused.
  bool start_paused = false;

  // Time to start playback as an offset from the beginning of the bag.
  rcutils_time_point_value_t start_offset = 0;

  bool disable_keyboard_controls = false;
  // keybindings
  KeyboardHandler::KeyCode pause_resume_toggle_key = KeyboardHandler::KeyCode::SPACE;
  KeyboardHandler::KeyCode play_next_key = KeyboardHandler::KeyCode::CURSOR_RIGHT;
  KeyboardHandler::KeyCode increase_rate_key = KeyboardHandler::KeyCode::CURSOR_UP;
  KeyboardHandler::KeyCode decrease_rate_key = KeyboardHandler::KeyCode::CURSOR_DOWN;

  // Timeout for waiting for all published messages to be acknowledged.
  // Negative value means that published messages do not need to be acknowledged.
  int64_t wait_acked_timeout = -1;

  // Disable to publish as loaned message
  bool disable_loan_message = false;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAY_OPTIONS_HPP_
