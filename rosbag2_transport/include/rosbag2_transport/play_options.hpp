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

  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides = {};
  bool loop = false;
  std::vector<std::string> topic_remapping_options = {};

  // Rate in Hz at which to publish to /clock.
  // 0 (or negative) means that no publisher will be created
  double clock_publish_frequency = 0.0;

  // Sleep before play. Negative durations invalid. Will delay at the beginning of each loop.
  rclcpp::Duration delay = rclcpp::Duration(0, 0);
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAY_OPTIONS_HPP_
