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

namespace rosbag2_transport {

namespace
{
rcl_interfaces::msg::ParameterDescriptor int_param_description(
  std::string description, int64_t min,
  int64_t max)
{
  rcl_interfaces::msg::ParameterDescriptor d{};
  rcl_interfaces::msg::IntegerRange r{};
  d.description = description;
  r.from_value = min;
  r.to_value = max;
  d.integer_range.push_back(r);
  return d;
}

rcl_interfaces::msg::ParameterDescriptor float_param_description(
  std::string description, float min,
  float max)
{
  rcl_interfaces::msg::ParameterDescriptor d{};
  rcl_interfaces::msg::FloatingPointRange r{};
  d.description = description;
  r.from_value = min;
  r.to_value = max;
  d.floating_point_range.push_back(r);
  return d;
}
}  // namespace

void declare_play_options_rw_params(std::shared_ptr<rclcpp::Node> nh, PlayOptions & po)
{
  static const std::vector<std::string> empty_str_list;

  auto desc_raqs = int_param_description(
    "Read ahead queue size (messages)",
    1,
    std::numeric_limits<int64_t>::max());
  auto read_ahead_queue_size_ = nh->declare_parameter<int64_t>(
    "play.read_ahead_queue_size",
    1000,
    desc_raqs);
  po.read_ahead_queue_size = static_cast<uint64_t>(read_ahead_queue_size_);

  po.node_prefix = nh->declare_parameter<std::string>(
    "play.node_prefix",
    "");

  auto desc_rate = float_param_description(
    "Playback rate (hz)",
    0.000001,
    std::numeric_limits<float>::max());
  po.rate = nh->declare_parameter<float>(
    "play.rate",
    1.0,
    desc_rate);

  po.topics_to_filter = nh->declare_parameter<std::vector<std::string>>(
    "play.topics_to_filter",
    empty_str_list);

  po.topics_regex_to_filter = nh->declare_parameter<std::string>(
    "play.topics_regex_to_filter",
    "");

  po.topics_regex_to_exclude = nh->declare_parameter<std::string>(
    "play.topics_regex_to_exclude",
    "");

  // TODO(roncapat)
  // std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides = {};

  po.loop = nh->declare_parameter<bool>(
    "play.loop",
    false);

  // TODO(roncapat): but I think it's worth to use classic CLI/launchfile remap instead
  // po.topic_remapping_options =
  //    nh->declare_parameter<std::vector<std::string>>(
  //       "play.topic_remapping_options", empty_str_list);

  po.clock_publish_frequency = nh->declare_parameter<double>(
    "play.clock_publish_frequency",
    0.0);

  po.clock_publish_on_topic_publish = nh->declare_parameter<bool>(
    "play.clock_publish_on_topic_publish",
    false);

  po.clock_trigger_topics = nh->declare_parameter<std::vector<std::string>>(
    "play.clock_trigger_topics",
    empty_str_list);

  auto playback_duration_ = nh->declare_parameter<double>(
    "play.playback_duration",
    -1);
  po.playback_duration = rclcpp::Duration::from_seconds(playback_duration_);

  po.playback_until_timestamp = nh->declare_parameter<int64_t>(
    "play.playback_until_timestamp",
    -1);

  po.start_paused = nh->declare_parameter<bool>(
    "play.start_paused",
    false);

  po.start_offset = nh->declare_parameter<int64_t>(
    "play.start_offset",
    0);

  po.disable_keyboard_controls = nh->declare_parameter<bool>(
    "play.disable_keyboard_controls",
    false);

  // TODO(roncapat): but these are never set in ros2 bag play verb. May we skip also here?
  // KeyboardHandler::KeyCode pause_resume_toggle_key = KeyboardHandler::KeyCode::SPACE;
  // KeyboardHandler::KeyCode play_next_key = KeyboardHandler::KeyCode::CURSOR_RIGHT;
  // KeyboardHandler::KeyCode increase_rate_key = KeyboardHandler::KeyCode::CURSOR_UP;
  // KeyboardHandler::KeyCode decrease_rate_key = KeyboardHandler::KeyCode::CURSOR_DOWN;

  po.wait_acked_timeout = nh->declare_parameter<int64_t>(
    "play.wait_acked_timeout",
    -1);

  po.disable_loan_message = nh->declare_parameter<bool>(
    "play.disable_loan_message",
    false);
}
}