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
  Node node;
  node["all"] = record_options.all;
  node["is_discovery_disabled"] = record_options.is_discovery_disabled;
  node["topics"] = record_options.topics;
  node["rmw_serialization_format"] = record_options.rmw_serialization_format;
  node["topic_polling_interval"] = record_options.topic_polling_interval;
  node["regex"] = record_options.regex;
  node["exclude"] = record_options.exclude;
  node["node_prefix"] = record_options.node_prefix;
  node["compression_mode"] = record_options.compression_mode;
  node["compression_format"] = record_options.compression_format;
  node["compression_queue_size"] = record_options.compression_queue_size;
  node["compression_threads"] = record_options.compression_threads;
  std::map<std::string, rosbag2_transport::Rosbag2QoS> qos_overrides(
    record_options.topic_qos_profile_overrides.begin(),
    record_options.topic_qos_profile_overrides.end());
  node["topic_qos_profile_overrides"] = qos_overrides;
  node["remappings"] = record_options.remappings;
  node["include_hidden_topics"] = record_options.include_hidden_topics;
  node["include_unpublished_topics"] = record_options.include_unpublished_topics;
  return node;
}

bool convert<rosbag2_transport::RecordOptions>::decode(
  const Node & node, rosbag2_transport::RecordOptions & record_options)
{
  optional_assign<bool>(node, "all", record_options.all);
  optional_assign<bool>(node, "is_discovery_disabled", record_options.is_discovery_disabled);
  optional_assign<std::vector<std::string>>(node, "topics", record_options.topics);
  optional_assign<std::string>(
    node, "rmw_serialization_format", record_options.rmw_serialization_format);
  optional_assign<std::chrono::milliseconds>(
    node, "topic_polling_interval", record_options.topic_polling_interval);
  optional_assign<std::string>(node, "regex", record_options.regex);
  optional_assign<std::string>(node, "exclude", record_options.exclude);
  optional_assign<std::string>(node, "node_prefix", record_options.node_prefix);
  optional_assign<std::string>(node, "compression_mode", record_options.compression_mode);
  optional_assign<std::string>(node, "compression_format", record_options.compression_format);
  optional_assign<uint64_t>(node, "compression_queue_size", record_options.compression_queue_size);
  optional_assign<uint64_t>(node, "compression_threads", record_options.compression_threads);

  // yaml-cpp doesn't implement unordered_map
  std::map<std::string, rosbag2_transport::Rosbag2QoS> qos_overrides;
  optional_assign<std::map<std::string, rosbag2_transport::Rosbag2QoS>>(
    node, "topic_qos_profile_overrides", qos_overrides);
  record_options.topic_qos_profile_overrides.insert(qos_overrides.begin(), qos_overrides.end());

  optional_assign<std::vector<std::string>>(node, "remappings", record_options.remappings);
  optional_assign<bool>(node, "include_hidden_topics", record_options.include_hidden_topics);
  optional_assign<bool>(
    node, "include_unpublished_topics",
    record_options.include_unpublished_topics);
  return true;
}

}  // namespace YAML
