// Copyright 2023 Patrick Roncagliolo.
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

#include <map>
#include <string>
#include <vector>

#include "rosbag2_transport/qos.hpp"
#include "rosbag2_transport/play_options.hpp"
#include "rosbag2_transport/utils/param_utils.hpp"
#include "rosbag2_transport/play_options_from_node_params.hpp"

namespace rosbag2_transport
{

PlayOptions get_play_options_from_node_params(std::shared_ptr<rclcpp::Node> node)
{
  PlayOptions play_options{};
  auto desc_raqs = param_utils::int_param_description(
    "Read ahead queue size (messages)",
    1,
    std::numeric_limits<int64_t>::max());
  auto read_ahead_queue_size_ = node->declare_parameter<int64_t>(
    "read_ahead_queue_size",
    1000,
    desc_raqs);
  play_options.read_ahead_queue_size = static_cast<uint64_t>(read_ahead_queue_size_);

  play_options.node_prefix = node->declare_parameter<std::string>(
    "node_prefix",
    "");

  auto desc_rate = param_utils::float_param_description(
    "Playback rate (hz)",
    0.000001,
    std::numeric_limits<float>::max());
  play_options.rate = node->declare_parameter<float>(
    "rate",
    1.0,
    desc_rate);

  play_options.topics_to_filter = node->declare_parameter<std::vector<std::string>>(
    "topics_to_filter",
    std::vector<std::string>());

  play_options.topics_regex_to_filter = node->declare_parameter<std::string>(
    "topics_regex_to_filter",
    "");

  play_options.topics_regex_to_exclude = node->declare_parameter<std::string>(
    "topics_regex_to_exclude",
    "");

  // TODO(roncapat)
  // std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides = {};

  play_options.loop = node->declare_parameter<bool>(
    "loop",
    false);

  auto topic_remapping_options = node->declare_parameter<std::vector<std::string>>(
    "topic_remapping_options",
    std::vector<std::string>());

  if (!topic_remapping_options.empty()) {
    RCLCPP_WARN(
      node->get_logger(),
      "Remappings shall be applied through standard CLI/Launch options."
      "'topic_remapping_options' content will be ignored.");
  }

  play_options.clock_publish_frequency = node->declare_parameter<double>(
    "clock_publish_frequency",
    0.0);

  play_options.clock_publish_on_topic_publish = node->declare_parameter<bool>(
    "clock_publish_on_topic_publish",
    false);

  play_options.clock_trigger_topics = node->declare_parameter<std::vector<std::string>>(
    "clock_trigger_topics",
    std::vector<std::string>());

  auto playback_duration_ = node->declare_parameter<double>(
    "playback_duration",
    -1);
  play_options.playback_duration = rclcpp::Duration::from_seconds(playback_duration_);

  play_options.playback_until_timestamp = node->declare_parameter<int64_t>(
    "playback_until_timestamp",
    -1);

  play_options.start_paused = node->declare_parameter<bool>(
    "start_paused",
    false);

  play_options.start_offset = node->declare_parameter<int64_t>(
    "start_offset",
    0);

  play_options.disable_keyboard_controls = node->declare_parameter<bool>(
    "disable_keyboard_controls",
    false);

  // TODO(roncapat): but these are never set in ros2 bag play verb.
  // KeyboardHandler::KeyCode pause_resume_toggle_key = KeyboardHandler::KeyCode::SPACE;
  // KeyboardHandler::KeyCode play_next_key = KeyboardHandler::KeyCode::CURSOR_RIGHT;
  // KeyboardHandler::KeyCode increase_rate_key = KeyboardHandler::KeyCode::CURSOR_UP;
  // KeyboardHandler::KeyCode decrease_rate_key = KeyboardHandler::KeyCode::CURSOR_DOWN;

  play_options.wait_acked_timeout = node->declare_parameter<int64_t>(
    "wait_acked_timeout",
    -1);

  play_options.disable_loan_message = node->declare_parameter<bool>(
    "disable_loan_message",
    false);

  return play_options;
}
}  // namespace rosbag2_transport
