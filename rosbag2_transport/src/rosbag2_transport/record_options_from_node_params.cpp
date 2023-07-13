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
#include "rosbag2_transport/utils/param_utils.hpp"
#include "rosbag2_transport/record_options_from_node_params.hpp"

namespace rosbag2_transport
{

RecordOptions get_record_options_from_node_params(std::shared_ptr<rclcpp::Node> node)
{
  RecordOptions record_options{};
  record_options.all = node->declare_parameter<bool>(
    "all",
    false);

  record_options.is_discovery_disabled = node->declare_parameter<bool>(
    "is_discovery_disabled",
    false);

  record_options.topics = node->declare_parameter<std::vector<std::string>>(
    "topics",
    std::vector<std::string>());

  record_options.rmw_serialization_format = node->declare_parameter<std::string>(
    "rmw_serialization_format",
    "");

  auto desc_tpi = param_utils::int_param_description(
    "Topic polling interval (ms)",
    1,
    std::numeric_limits<int64_t>::max());
  auto topic_polling_interval_ = node->declare_parameter<int64_t>(
    "topic_polling_interval",
    100,
    desc_tpi);
  record_options.topic_polling_interval = std::chrono::milliseconds{topic_polling_interval_};

  record_options.regex = node->declare_parameter<std::string>(
    "regex",
    "");

  record_options.exclude = node->declare_parameter<std::string>(
    "exclude",
    "");

  record_options.node_prefix = node->declare_parameter<std::string>(
    "node_prefix",
    "");

  record_options.compression_mode = node->declare_parameter<std::string>(
    "compression_mode",
    "");

  record_options.compression_format = node->declare_parameter<std::string>(
    "compression_format",
    "");

  auto desc_cqs = param_utils::int_param_description(
    "Compression queue size (messages)",
    1,
    std::numeric_limits<int64_t>::max());
  auto compression_queue_size_ = node->declare_parameter<int64_t>(
    "compression_queue_size",
    1,
    desc_cqs);
  record_options.compression_queue_size = static_cast<uint64_t>(compression_queue_size_);

  auto desc_cts = param_utils::int_param_description(
    "Compression threads",
    0,
    std::numeric_limits<int64_t>::max());
  auto compression_threads_ = node->declare_parameter<int64_t>(
    "compression_threads",
    0,
    desc_cts);
  record_options.compression_threads = static_cast<uint64_t>(compression_threads_);

  // TODO(roncapat)
  // std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides{};

  record_options.include_hidden_topics = node->declare_parameter<bool>(
    "include_hidden_topics",
    false);

  record_options.include_unpublished_topics = node->declare_parameter<bool>(
    "include_unpublished_topics",
    false);

  record_options.ignore_leaf_topics = node->declare_parameter<bool>(
    "ignore_leaf_topics",
    false);

  record_options.start_paused = node->declare_parameter<bool>(
    "start_paused",
    false);

  record_options.use_sim_time = node->declare_parameter<bool>(
    "use_sim_time",
    false);


  if (record_options.use_sim_time && record_options.is_discovery_disabled) {
    throw std::invalid_argument(
            "'use_sim_time' and 'is_discovery_disabled' both set, but are incompatible settings. "
            "The `/clock` topic needs to be discovered to record with sim time.");
  }
  return record_options;
}
}  // namespace rosbag2_transport
