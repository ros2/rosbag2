// Copyright 2020-2021, Robotec.ai sp. z o.o.
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

#ifndef ROSBAG2_PERFORMANCE_BENCHMARKING__CONFIG_UTILS_HPP_
#define ROSBAG2_PERFORMANCE_BENCHMARKING__CONFIG_UTILS_HPP_

#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "rosbag2_performance_benchmarking/bag_config.hpp"
#include "rosbag2_performance_benchmarking/publisher_group_config.hpp"

namespace config_utils
{

constexpr std::string_view DEFAULT_MESSAGE_TYPE =
  "rosbag2_performance_benchmarking_msgs::msg::ByteArray";

/// Helper function to fill group_config with qos parameters
void load_qos_configuration(
  rclcpp::Node & node,
  PublisherGroupConfig & group_config,
  const std::string & group_prefix);

/// Acquires the parameter determining whether to wait for subscriber
bool wait_for_subscriptions_from_node_parameters(rclcpp::Node & node);

/// Acquires publisher parameters from the node
std::vector<PublisherGroupConfig> publisher_groups_from_node_parameters(
  rclcpp::Node & node);

/// Acquires bag parameters from the node
BagConfig bag_config_from_node_parameters(rclcpp::Node & node);

}  // namespace config_utils

#endif  // ROSBAG2_PERFORMANCE_BENCHMARKING__CONFIG_UTILS_HPP_
