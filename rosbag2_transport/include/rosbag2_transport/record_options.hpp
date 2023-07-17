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

#ifndef ROSBAG2_TRANSPORT__RECORD_OPTIONS_HPP_
#define ROSBAG2_TRANSPORT__RECORD_OPTIONS_HPP_

#include <chrono>
#include <memory>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "keyboard_handler/keyboard_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{
struct RecordOptions
{
public:
  bool all = false;
  bool is_discovery_disabled = false;
  std::vector<std::string> topics;
  std::string rmw_serialization_format;
  std::chrono::milliseconds topic_polling_interval{100};
  std::string regex = "";
  std::string exclude = "";
  std::string node_prefix = "";
  std::string compression_mode = "";
  std::string compression_format = "";
  uint64_t compression_queue_size = 1;
  uint64_t compression_threads = 0;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides{};
  bool include_hidden_topics = false;
  bool include_unpublished_topics = false;
  bool ignore_leaf_topics = false;
  bool start_paused = false;
  bool use_sim_time = false;
};

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

}  // namespace rosbag2_transport

namespace YAML
{
template<>
struct ROSBAG2_TRANSPORT_PUBLIC convert<rosbag2_transport::RecordOptions>
{
  static Node encode(const rosbag2_transport::RecordOptions & storage_options);
  static bool decode(const Node & node, rosbag2_transport::RecordOptions & storage_options);
};
}  // namespace YAML

#endif  // ROSBAG2_TRANSPORT__RECORD_OPTIONS_HPP_
