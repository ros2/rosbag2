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

void declare_record_options_rw_params(std::shared_ptr<rclcpp::Node> nh, RecordOptions & ro)
{
  static const std::vector<std::string> empty_str_list;

  ro.all = nh->declare_parameter<bool>(
    "record.all",
    false);

  ro.is_discovery_disabled = nh->declare_parameter<bool>(
    "record.is_discovery_disabled",
    false);

  ro.topics = nh->declare_parameter<std::vector<std::string>>(
    "record.topics",
    empty_str_list);

  ro.rmw_serialization_format = nh->declare_parameter<std::string>(
    "record.rmw_serialization_format",
    "");

  auto desc_tpi = int_param_description(
    "Topic polling interval (ms)",
    1,
    std::numeric_limits<int64_t>::max());
  auto topic_polling_interval_ = nh->declare_parameter<int64_t>(
    "record.topic_polling_interval",
    100,
    desc_tpi);
  ro.topic_polling_interval = std::chrono::milliseconds{topic_polling_interval_};

  ro.regex = nh->declare_parameter<std::string>(
    "record.regex",
    "");

  ro.exclude = nh->declare_parameter<std::string>(
    "record.exclude",
    "");

  ro.node_prefix = nh->declare_parameter<std::string>(
    "record.node_prefix",
    "");

  ro.compression_mode = nh->declare_parameter<std::string>(
    "record.compression_mode",
    "");

  ro.compression_format = nh->declare_parameter<std::string>(
    "record.compression_format",
    "");

  auto desc_cqs = int_param_description(
    "Compression queue size (messages)",
    1,
    std::numeric_limits<int64_t>::max());
  auto compression_queue_size_ = nh->declare_parameter<int64_t>(
    "record.compression_queue_size",
    1,
    desc_cqs);
  ro.compression_queue_size = std::static_cast<uint64_t>(compression_queue_size_);

  auto desc_cts = int_param_description(
    "Compression threads",
    0,
    std::numeric_limits<int64_t>::max());
  auto compression_threads_ = nh->declare_parameter<int64_t>(
    "record.compression_threads",
    0,
    desc_cts);
  ro.compression_threads = std::static_cast<uint64_t>(compression_threads_);

  // TODO(roncapat)
  // std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides{};

  ro.include_hidden_topics = nh->declare_parameter<bool>(
    "record.include_hidden_topics",
    false);

  ro.include_unpublished_topics = nh->declare_parameter<bool>(
    "record.include_unpublished_topics",
    false);

  ro.ignore_leaf_topics = nh->declare_parameter<bool>(
    "record.ignore_leaf_topics",
    false);

  ro.start_paused = nh->declare_parameter<bool>(
    "record.start_paused",
    false);

  ro.use_sim_time = nh->declare_parameter<bool>(
    "record.use_sim_time",
    false);
}
}

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

  optional_assign<bool>(node, "include_hidden_topics", record_options.include_hidden_topics);
  optional_assign<bool>(
    node, "include_unpublished_topics",
    record_options.include_unpublished_topics);
  return true;
}

}  // namespace YAML
