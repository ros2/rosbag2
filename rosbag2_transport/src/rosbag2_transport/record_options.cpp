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

#include "rosbag2_transport/qos.hpp"
#include "rosbag2_transport/record_options.hpp"

namespace YAML
{

template<>
struct convert<std::chrono::milliseconds>
{
  static Node encode(const std::chrono::milliseconds & millis)
  {
    Node node{millis.count()};
    return node;
  }

  static bool decode(const Node & node, std::chrono::milliseconds & millis)
  {
    millis = std::chrono::milliseconds{node.as<int>()};
    return true;
  }
};

Node convert<rosbag2_transport::RecordOptions>::encode(
  const rosbag2_transport::RecordOptions & record_options)
{
  // Structured binding check - this line will fail to compile if fields are added/removed to
  // RecordOptions without updating this function.
  // Note: Please don't forget to update the test case `test_yaml_serialization_deserialization`
  // in `test_record_options.cpp` when updating the fields in RecordOptions.
  auto & [all, is_discovery_disabled, topics,
    rmw_serialization_format,
    topic_polling_interval, regex, exclude, node_prefix,
    compression_mode, compression_format, compression_queue_size, compression_threads,
    topic_qos_profile_overrides,
    include_hidden_topics, include_unpublished_topics, ignore_leaf_topics,
    start_paused, use_sim_time] = record_options;
  Node node;
  node["all"] = all;
  node["is_discovery_disabled"] = is_discovery_disabled;
  node["topics"] = topics;
  node["rmw_serialization_format"] = rmw_serialization_format;
  node["topic_polling_interval"] = topic_polling_interval;
  node["regex"] = regex;
  node["exclude"] = exclude;
  node["node_prefix"] = node_prefix;
  node["compression_mode"] = compression_mode;
  node["compression_format"] = compression_format;
  node["compression_queue_size"] = compression_queue_size;
  node["compression_threads"] = compression_threads;
  std::map<std::string, rosbag2_transport::Rosbag2QoS> qos_overrides(
    topic_qos_profile_overrides.begin(), topic_qos_profile_overrides.end());
  node["topic_qos_profile_overrides"] = qos_overrides;
  node["include_hidden_topics"] = include_hidden_topics;
  node["include_unpublished_topics"] = include_unpublished_topics;
  node["ignore_leaf_topics"] = ignore_leaf_topics;
  node["start_paused"] = start_paused;
  node["use_sim_time"] = use_sim_time;
  return node;
}

bool convert<rosbag2_transport::RecordOptions>::decode(
  const Node & node, rosbag2_transport::RecordOptions & record_options)
{
  // Structured binding check - this line will fail to compile if fields are added/removed to
  // RecordOptions without updating this function.
  // Note: Please don't forget to update the test case `test_yaml_serialization_deserialization`
  // in `test_record_options.cpp` when updating the fields in RecordOptions.
  auto & [all, is_discovery_disabled, topics,
    rmw_serialization_format,
    topic_polling_interval, regex, exclude, node_prefix,
    compression_mode, compression_format, compression_queue_size, compression_threads,
    topic_qos_profile_overrides,
    include_hidden_topics, include_unpublished_topics, ignore_leaf_topics,
    start_paused, use_sim_time] = record_options;

  optional_assign<bool>(node, "all", all);
  optional_assign<bool>(node, "is_discovery_disabled", is_discovery_disabled);
  optional_assign<std::vector<std::string>>(node, "topics", topics);
  optional_assign<std::string>(node, "rmw_serialization_format", rmw_serialization_format);
  optional_assign<std::chrono::milliseconds>(
    node, "topic_polling_interval", topic_polling_interval);

  optional_assign<std::string>(node, "regex", regex);
  // Map exclude to the "exclude_regex" for backward compatability.
  optional_assign<std::string>(node, "exclude", exclude);
  optional_assign<std::string>(node, "exclude_regex", exclude);
  optional_assign<std::string>(node, "node_prefix", node_prefix);
  optional_assign<std::string>(node, "compression_mode", compression_mode);
  optional_assign<std::string>(node, "compression_format", compression_format);
  optional_assign<uint64_t>(node, "compression_queue_size", compression_queue_size);
  optional_assign<uint64_t>(node, "compression_threads", compression_threads);

  // yaml-cpp doesn't implement unordered_map
  std::map<std::string, rosbag2_transport::Rosbag2QoS> qos_overrides;
  optional_assign<std::map<std::string, rosbag2_transport::Rosbag2QoS>>(
    node, "topic_qos_profile_overrides", qos_overrides);
  topic_qos_profile_overrides.insert(qos_overrides.begin(), qos_overrides.end());

  optional_assign<bool>(node, "include_hidden_topics", include_hidden_topics);
  optional_assign<bool>(node, "include_unpublished_topics", include_unpublished_topics);
  optional_assign<bool>(node, "ignore_leaf_topics", ignore_leaf_topics);
  optional_assign<bool>(node, "start_paused", start_paused);
  optional_assign<bool>(node, "use_sim_time", use_sim_time);
  return true;
}

}  // namespace YAML
