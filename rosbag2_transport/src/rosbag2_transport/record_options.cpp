// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include "rosbag2_storage/qos.hpp"
#include "rosbag2_transport/play_options.hpp"
#include "rosbag2_transport/record_options.hpp"

namespace YAML
{

Node convert<rosbag2_transport::RecordOptions>::encode(
  const rosbag2_transport::RecordOptions & record_options)
{
  // Structured binding check - this line will fail to compile if fields are added/removed to
  // RecordOptions without updating this function.
  // Note: Please don't forget to update the test case `test_yaml_serialization_deserialization`
  // in `test_record_options.cpp` when updating the fields in RecordOptions.
  auto & [all_topics, all_services, all_actions, is_discovery_disabled,
    topics, topic_types, services, actions,
    exclude_topics, exclude_topic_types, exclude_service_events, exclude_actions,
    rmw_serialization_format, input_serialization_format, output_serialization_format,
    topic_polling_interval, regex, exclude_regex, node_prefix,
    compression_mode, compression_format, compression_queue_size, compression_threads,
    compression_threads_priority, topic_qos_profile_overrides,
    include_hidden_topics, include_unpublished_topics, ignore_leaf_topics,
    start_paused, use_sim_time, static_topics_uri, disable_keyboard_controls,
    statistics_max_publishing_rate, repeat_transient_local_messages,
    repeat_all_transient_local_depth] = record_options;
  Node node;
  node["all_topics"] = all_topics;
  node["all_services"] = all_services;
  node["all_actions"] = all_actions;
  node["is_discovery_disabled"] = is_discovery_disabled;
  node["topics"] = topics;
  node["topic_types"] = topic_types;
  node["services"] = services;
  node["actions"] = actions;
  node["exclude_topics"] = exclude_topics;
  node["exclude_topic_types"] = exclude_topic_types;
  node["exclude_services"] = exclude_service_events;
  node["exclude_actions"] = exclude_actions;
  node["rmw_serialization_format"] = rmw_serialization_format;
  node["input_serialization_format"] = input_serialization_format;
  node["output_serialization_format"] = output_serialization_format;
  node["topic_polling_interval"] = topic_polling_interval;
  node["regex"] = regex;
  node["exclude_regex"] = exclude_regex;
  node["node_prefix"] = node_prefix;
  node["compression_mode"] = compression_mode;
  node["compression_format"] = compression_format;
  node["compression_queue_size"] = compression_queue_size;
  node["compression_threads"] = compression_threads;
  node["compression_threads_priority"] = compression_threads_priority;
  node["topic_qos_profile_overrides"] =
    convert<std::unordered_map<std::string, rclcpp::QoS>>::encode(topic_qos_profile_overrides);
  node["include_hidden_topics"] = include_hidden_topics;
  node["include_unpublished_topics"] = include_unpublished_topics;
  node["ignore_leaf_topics"] = ignore_leaf_topics;
  node["start_paused"] = start_paused;
  node["use_sim_time"] = use_sim_time;
  node["static_topics_uri"] = static_topics_uri;
  node["disable_keyboard_controls"] = disable_keyboard_controls;
  node["statistics_max_publishing_rate"] = statistics_max_publishing_rate;
  node["repeat_transient_local_messages"] = repeat_transient_local_messages;
  node["repeat_all_transient_local_depth"] = repeat_all_transient_local_depth;
  return node;
}

bool convert<rosbag2_transport::RecordOptions>::decode(
  const Node & node, rosbag2_transport::RecordOptions & record_options, int version)
{
  // Structured binding check - this line will fail to compile if fields are added/removed to
  // RecordOptions without updating this function.
  // Note: Please don't forget to update the test case `test_yaml_serialization_deserialization`
  // in `test_record_options.cpp` when updating the fields in RecordOptions.
  auto & [all_topics, all_services, all_actions, is_discovery_disabled,
    topics, topic_types, services, actions,
    exclude_topics, exclude_topic_types, exclude_service_events, exclude_actions,
    rmw_serialization_format, input_serialization_format, output_serialization_format,
    topic_polling_interval, regex, exclude_regex, node_prefix,
    compression_mode, compression_format, compression_queue_size, compression_threads,
    compression_threads_priority, topic_qos_profile_overrides,
    include_hidden_topics, include_unpublished_topics, ignore_leaf_topics,
    start_paused, use_sim_time, static_topics_uri, disable_keyboard_controls,
    statistics_max_publishing_rate, repeat_transient_local_messages,
    repeat_all_transient_local_depth] = record_options;

  optional_assign<bool>(node, "all_topics", all_topics);
  optional_assign<bool>(node, "all_services", all_services);
  optional_assign<bool>(node, "all_actions", all_actions);
  bool record_options_all{false};  // Parse `all` for backward compatability and convenient usage
  optional_assign<bool>(node, "all", record_options_all);
  if (record_options_all) {
    all_topics = true;
    all_services = true;
    all_actions = true;
  }

  optional_assign<bool>(node, "is_discovery_disabled", is_discovery_disabled);
  optional_assign<std::vector<std::string>>(node, "topics", topics);
  optional_assign<std::vector<std::string>>(node, "topic_types", topic_types);
  optional_assign<std::vector<std::string>>(node, "services", services);
  optional_assign<std::vector<std::string>>(node, "actions", actions);
  optional_assign<std::vector<std::string>>(node, "exclude_topics", exclude_topics);
  optional_assign<std::vector<std::string>>(node, "exclude_topic_types", exclude_topic_types);
  optional_assign<std::vector<std::string>>(node, "exclude_services", exclude_service_events);
  optional_assign<std::vector<std::string>>(node, "exclude_actions", exclude_actions);
  optional_assign<std::string>(
    node, "rmw_serialization_format", rmw_serialization_format);
  optional_assign<std::string>(
    node, "input_serialization_format", input_serialization_format);
  optional_assign<std::string>(
    node, "output_serialization_format", output_serialization_format);

  optional_assign<std::chrono::milliseconds>(
    node, "topic_polling_interval", topic_polling_interval);

  optional_assign<std::string>(node, "regex", regex);
  // Map exclude to the "exclude_regex" for backward compatability.
  optional_assign<std::string>(node, "exclude", exclude_regex);
  optional_assign<std::string>(node, "exclude_regex", exclude_regex);
  optional_assign<std::string>(node, "node_prefix", node_prefix);
  optional_assign<std::string>(node, "compression_mode", compression_mode);
  optional_assign<std::string>(node, "compression_format", compression_format);
  optional_assign<uint64_t>(node, "compression_queue_size", compression_queue_size);
  optional_assign<uint64_t>(node, "compression_threads", compression_threads);
  optional_assign<int32_t>(node, "compression_threads_priority", compression_threads_priority);

  std::unordered_map<std::string, rclcpp::QoS> qos_overrides;
  if (node["topic_qos_profile_overrides"]) {
    qos_overrides = YAML::decode_for_version<std::unordered_map<std::string, rclcpp::QoS>>(
      node["topic_qos_profile_overrides"], version);
  }
  topic_qos_profile_overrides = qos_overrides;

  optional_assign<bool>(node, "include_hidden_topics", include_hidden_topics);
  optional_assign<bool>(node, "include_unpublished_topics", include_unpublished_topics);
  optional_assign<bool>(node, "ignore_leaf_topics", ignore_leaf_topics);
  optional_assign<bool>(node, "start_paused", start_paused);
  optional_assign<bool>(node, "use_sim_time", use_sim_time);
  optional_assign<std::string>(node, "static_topics_uri", static_topics_uri);
  optional_assign<bool>(node, "disable_keyboard_controls", disable_keyboard_controls);
  optional_assign<float>(node, "statistics_max_publishing_rate", statistics_max_publishing_rate);
  optional_assign<std::unordered_map<std::string, size_t>>(
    node, "repeat_transient_local_messages", repeat_transient_local_messages);
  optional_assign<uint32_t>(node, "repeat_all_transient_local_depth",
    repeat_all_transient_local_depth);
  return true;
}

}  // namespace YAML
