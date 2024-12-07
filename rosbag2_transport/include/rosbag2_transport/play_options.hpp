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

#include <chrono>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "keyboard_handler/keyboard_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{
enum class ServiceRequestsSource : int8_t
{
  SERVICE_INTROSPECTION = 0,
  CLIENT_INTROSPECTION = 1
};

struct PlayOptions
{
public:
  size_t read_ahead_queue_size = 1000;
  std::string node_prefix = "";
  float rate = 1.0;

  // Topic names to whitelist when playing a bag.
  // Only messages matching these specified topics will be played.
  // If list is empty, the filter is ignored and all messages of topics are played.
  std::vector<std::string> topics_to_filter = {};

  // Service names (service event topic names) to whitelist when playing a bag.
  // Only messages matching these specified services will be played.
  // If list is empty, the filter is ignored and all messages of services are played.
  std::vector<std::string> services_to_filter = {};

  // Regular expression of topic names and service name to whitelist when playing a bag.
  // Only messages matching these specified topics and services will be played.
  // If list is empty, the filter is ignored and all messages are played.
  std::string regex_to_filter = "";

  // List of topic names to exclude when playing a bag.
  // Only messages not matching these specified topics will be played.
  std::vector<std::string> exclude_topics_to_filter = {};

  // List of service names (service event topic names) to exclude when playing a bag.
  // Only messages not matching these specified services will be played.
  std::vector<std::string> exclude_services_to_filter = {};

  // Regular expression of topic names and service name to exclude when playing a bag.
  // Only messages not matching these specified topics and services will be played.
  std::string exclude_regex_to_filter = "";

  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides = {};
  bool loop = false;
  std::vector<std::string> topic_remapping_options = {};

  // Rate in Hz at which to publish to /clock.
  // 0 (or negative) means that no publisher will be created
  double clock_publish_frequency = 0.0;

  // Enable publishing to /clock when a replayed topic is published
  bool clock_publish_on_topic_publish{false};

  // If clock_publish_on_topic_publish is true, list of topics that will trigger
  // a /clock update to be published. If list is empty, all topics will act as a trigger
  std::vector<std::string> clock_trigger_topics = {};

  // Sleep before play. Negative durations invalid. Loops are not affected.
  rclcpp::Duration delay = rclcpp::Duration(0, 0);

  // Determines the maximum duration of the playback. Negative durations will make the playback to
  // not stop. Default configuration makes the player to not stop execution.
  // When positive, the maximum of `play_until_timestamp` and the one that this attribute yields
  // will be used to determine which one stops playback execution.
  rclcpp::Duration playback_duration = rclcpp::Duration(-1, 0);

  // Determines the timestamp at which the playback will stop. Negative timestamps will make the
  // playback to not stop. Default configuration makes the player to not stop execution.
  // When positive, the maximum of the effective time that `playback_duration` yields and this
  // attribute will be used to determine which one stops playback execution.
  rcutils_time_point_value_t playback_until_timestamp = -1;

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

  // Publish service requests instead of service events
  bool publish_service_requests = false;

  // The source of the service request
  ServiceRequestsSource service_requests_source = ServiceRequestsSource::SERVICE_INTROSPECTION;
};

}  // namespace rosbag2_transport

namespace YAML
{
template<>
struct ROSBAG2_TRANSPORT_PUBLIC convert<rosbag2_transport::PlayOptions>
{
  static Node encode(const rosbag2_transport::PlayOptions & play_options);
  static bool decode(const Node & node, rosbag2_transport::PlayOptions & play_options);
};

}  // namespace YAML

#endif  // ROSBAG2_TRANSPORT__PLAY_OPTIONS_HPP_
