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
  Node node;
  node["all_topics"] = record_options.all_topics;
  node["all_services"] = record_options.all_services;
  node["is_discovery_disabled"] = record_options.is_discovery_disabled;
  node["topics"] = record_options.topics;
  node["topic_types"] = record_options.topic_types;
  node["exclude_topic_types"] = record_options.exclude_topic_types;
  node["services"] = record_options.services;
  node["rmw_serialization_format"] = record_options.rmw_serialization_format;
  node["topic_polling_interval"] = record_options.topic_polling_interval;
  node["regex"] = record_options.regex;
  node["exclude_regex"] = record_options.exclude_regex;
  node["exclude_topics"] = record_options.exclude_topics;
  node["exclude_services"] = record_options.exclude_service_events;
  node["node_prefix"] = record_options.node_prefix;
  node["compression_mode"] = record_options.compression_mode;
  node["compression_format"] = record_options.compression_format;
  node["compression_queue_size"] = record_options.compression_queue_size;
  node["compression_threads"] = record_options.compression_threads;
  node["compression_threads_priority"] = record_options.compression_threads_priority;
  node["topic_qos_profile_overrides"] =
    convert<std::unordered_map<std::string, rclcpp::QoS>>::encode(
    record_options.topic_qos_profile_overrides);
  node["include_hidden_topics"] = record_options.include_hidden_topics;
  node["include_unpublished_topics"] = record_options.include_unpublished_topics;
  node["disable_keyboard_controls"] = record_options.disable_keyboard_controls;
  return node;
}

bool convert<rosbag2_transport::RecordOptions>::decode(
  const Node & node, rosbag2_transport::RecordOptions & record_options, int version)
{
  optional_assign<bool>(node, "all_topics", record_options.all_topics);
  optional_assign<bool>(node, "all_services", record_options.all_services);
  bool record_options_all{false};  // Parse `all` for backward compatability and convenient usage
  optional_assign<bool>(node, "all", record_options_all);
  if (record_options_all) {
    record_options.all_topics = true;
    record_options.all_services = true;
  }

  optional_assign<bool>(node, "is_discovery_disabled", record_options.is_discovery_disabled);
  optional_assign<std::vector<std::string>>(node, "topics", record_options.topics);
  optional_assign<std::vector<std::string>>(node, "topic_types", record_options.topic_types);
  optional_assign<std::vector<std::string>>(node, "services", record_options.services);
  optional_assign<std::string>(
    node, "rmw_serialization_format", record_options.rmw_serialization_format);

  optional_assign<std::chrono::milliseconds>(
    node, "topic_polling_interval", record_options.topic_polling_interval);

  optional_assign<std::string>(node, "regex", record_options.regex);
  // Map exclude to the "exclude_regex" for backward compatability.
  optional_assign<std::string>(node, "exclude", record_options.exclude_regex);
  optional_assign<std::string>(node, "exclude_regex", record_options.exclude_regex);
  optional_assign<std::vector<std::string>>(node, "exclude_topics", record_options.exclude_topics);
  optional_assign<std::vector<std::string>>(
    node, "exclude_topic_types",
    record_options.exclude_topic_types);
  optional_assign<std::vector<std::string>>(
    node, "exclude_services", record_options.exclude_service_events);
  optional_assign<std::string>(node, "node_prefix", record_options.node_prefix);
  optional_assign<std::string>(node, "compression_mode", record_options.compression_mode);
  optional_assign<std::string>(node, "compression_format", record_options.compression_format);
  optional_assign<uint64_t>(node, "compression_queue_size", record_options.compression_queue_size);
  optional_assign<uint64_t>(node, "compression_threads", record_options.compression_threads);
  optional_assign<int32_t>(
    node, "compression_threads_priority",
    record_options.compression_threads_priority);

  std::unordered_map<std::string, rclcpp::QoS> qos_overrides;
  if (node["topic_qos_profile_overrides"]) {
    qos_overrides = YAML::decode_for_version<std::unordered_map<std::string, rclcpp::QoS>>(
      node["topic_qos_profile_overrides"], version);
  }
  record_options.topic_qos_profile_overrides = qos_overrides;

  optional_assign<bool>(node, "include_hidden_topics", record_options.include_hidden_topics);
  optional_assign<bool>(
    node, "include_unpublished_topics",
    record_options.include_unpublished_topics);
  optional_assign<bool>(
    node, "disable_keyboard_controls",
    record_options.disable_keyboard_controls);
  return true;
}

}  // namespace YAML
