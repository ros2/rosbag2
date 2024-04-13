// Copyright 2023 Patrick Roncagliolo. All Rights Reserved.
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

#include "rclcpp/duration.hpp"
#include "rosbag2_storage/qos.hpp"
#include "rosbag2_transport/play_options.hpp"

namespace YAML
{

Node convert<rosbag2_transport::PlayOptions>::encode(
  const rosbag2_transport::PlayOptions & play_options)
{
  Node node;
  node["read_ahead_queue_size"] = play_options.read_ahead_queue_size;
  node["node_prefix"] = play_options.node_prefix;
  node["rate"] = play_options.rate;
  node["topics_to_filter"] = play_options.topics_to_filter;
  node["services_to_filter"] = play_options.services_to_filter;
  node["regex_to_filter"] = play_options.regex_to_filter;
  node["exclude_regex_to_filter"] = play_options.exclude_regex_to_filter;
  node["exclude_topics"] = play_options.exclude_topics_to_filter;
  node["exclude_services"] = play_options.exclude_services_to_filter;
  node["topic_qos_profile_overrides"] =
    YAML::convert<std::unordered_map<std::string, rclcpp::QoS>>::encode(
    play_options.topic_qos_profile_overrides);
  node["loop"] = play_options.loop;
  node["topic_remapping_options"] = play_options.topic_remapping_options;
  node["clock_publish_frequency"] = play_options.clock_publish_frequency;
  node["clock_publish_on_topic_publish"] = play_options.clock_publish_on_topic_publish;
  node["clock_trigger_topics"] = play_options.clock_trigger_topics;
  node["delay"] = play_options.delay;
  node["playback_duration"] = play_options.playback_duration;
  node["playback_until_timestamp"] = YAML::convert<rclcpp::Duration>::encode(
    std::chrono::nanoseconds(play_options.playback_until_timestamp));

  node["start_paused"] = play_options.start_paused;
  node["start_offset"] = YAML::convert<rclcpp::Duration>::encode(
    std::chrono::nanoseconds(play_options.start_offset));

  node["disable_keyboard_controls"] = play_options.disable_keyboard_controls;
  node["wait_acked_timeout"] = YAML::convert<rclcpp::Duration>::encode(
    std::chrono::nanoseconds(play_options.wait_acked_timeout));

  node["disable_loan_message"] = play_options.disable_loan_message;

  return node;
}

bool convert<rosbag2_transport::PlayOptions>::decode(
  const Node & node, rosbag2_transport::PlayOptions & play_options)
{
  optional_assign<size_t>(node, "read_ahead_queue_size", play_options.read_ahead_queue_size);
  optional_assign<std::string>(node, "node_prefix", play_options.node_prefix);
  optional_assign<float>(node, "rate", play_options.rate);
  optional_assign<std::vector<std::string>>(
    node, "topics_to_filter", play_options.topics_to_filter);
  optional_assign<std::vector<std::string>>(
    node, "services_to_filter", play_options.services_to_filter);
  optional_assign<std::string>(node, "regex_to_filter", play_options.regex_to_filter);
  optional_assign<std::string>(
    node, "exclude_regex_to_filter",
    play_options.exclude_regex_to_filter);
  optional_assign<std::vector<std::string>>(
    node, "exclude_topics", play_options.exclude_topics_to_filter);
  optional_assign<std::vector<std::string>>(
    node, "exclude_services", play_options.exclude_services_to_filter);

  optional_assign<std::unordered_map<std::string, rclcpp::QoS>>(
    node, "topic_qos_profile_overrides", play_options.topic_qos_profile_overrides);

  optional_assign<double>(node, "clock_publish_frequency", play_options.clock_publish_frequency);

  optional_assign<bool>(
    node, "clock_publish_on_topic_publish", play_options.clock_publish_on_topic_publish);

  optional_assign<std::vector<std::string>>(
    node, "clock_trigger_topics", play_options.clock_trigger_topics);

  optional_assign<rclcpp::Duration>(node, "delay", play_options.delay);
  optional_assign<rclcpp::Duration>(node, "playback_duration", play_options.playback_duration);

  rclcpp::Duration playback_until_timestamp(
    std::chrono::nanoseconds(play_options.playback_until_timestamp));
  optional_assign<rclcpp::Duration>(node, "playback_until_timestamp", playback_until_timestamp);
  play_options.playback_until_timestamp = playback_until_timestamp.nanoseconds();

  optional_assign<bool>(node, "start_paused", play_options.start_paused);

  rclcpp::Duration start_offset(std::chrono::nanoseconds(play_options.start_offset));
  optional_assign<rclcpp::Duration>(node, "start_offset", start_offset);
  play_options.start_offset = start_offset.nanoseconds();

  optional_assign<bool>(node, "disable_keyboard_controls", play_options.disable_keyboard_controls);

  rclcpp::Duration wait_acked_timeout(std::chrono::nanoseconds(play_options.wait_acked_timeout));
  optional_assign<rclcpp::Duration>(node, "wait_acked_timeout", wait_acked_timeout);
  play_options.wait_acked_timeout = wait_acked_timeout.nanoseconds();

  optional_assign<bool>(node, "disable_loan_message", play_options.disable_loan_message);

  return true;
}

}  // namespace YAML
