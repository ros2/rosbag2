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
#include "rosbag2_performance_benchmarking/producer_config.hpp"
#include "rosbag2_performance_benchmarking/publisher_group_config.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

namespace config_utils
{

void load_qos_configuration(
  rclcpp::Node & node,
  PublisherGroupConfig & group_config,
  const std::string & group_prefix)
{
  auto qos_prefix = group_prefix + ".qos";
  node.declare_parameter(qos_prefix + ".qos_depth");
  node.declare_parameter(qos_prefix + ".qos_reliability");
  node.declare_parameter(qos_prefix + ".qos_durability");

  uint qos_depth = 10;
  std::string qos_reliability, qos_durability;
  node.get_parameter(qos_prefix + ".qos_depth", qos_depth);
  node.get_parameter(qos_prefix + ".qos_reliability", qos_reliability);
  node.get_parameter(qos_prefix + ".qos_durability", qos_durability);

  group_config.qos.keep_last(qos_depth);
  // TODO(adamdbrw) - error handling / map string to function
  if (qos_reliability == "reliable") {group_config.qos.reliable();}
  if (qos_reliability == "best_effort") {group_config.qos.best_effort();}
  if (qos_reliability == "transient_local") {group_config.qos.transient_local();}
  if (qos_reliability == "volatile") {group_config.qos.durability_volatile();}
}

std::vector<PublisherGroupConfig> load_from_node_parameters(
  rclcpp::Node & node)
{
  std::vector<PublisherGroupConfig> configurations;
  std::vector<std::string> publisher_groups;
  const std::string parameters_ns = "publishers";
  node.declare_parameter(parameters_ns + ".publisher_groups");
  node.get_parameter(parameters_ns + ".publisher_groups", publisher_groups);
  for (const auto & group_name : publisher_groups) {
    auto group_prefix = parameters_ns + "." + group_name;
    node.declare_parameter(group_prefix + ".publishers_count");
    node.declare_parameter(group_prefix + ".topic_root");
    node.declare_parameter(group_prefix + ".msg_size_bytes");
    node.declare_parameter(group_prefix + ".msg_count_each");
    node.declare_parameter(group_prefix + ".rate_hz");

    PublisherGroupConfig group_config;
    node.get_parameter(
      group_prefix + ".publishers_count",
      group_config.count);
    node.get_parameter(
      group_prefix + ".topic_root",
      group_config.topic_root);
    node.get_parameter(
      group_prefix + ".msg_size_bytes",
      group_config.producer_config.message_size);
    node.get_parameter(
      group_prefix + ".msg_count_each",
      group_config.producer_config.max_count);
    node.get_parameter(
      group_prefix + ".rate_hz",
      group_config.producer_config.frequency);
    config_utils::load_qos_configuration(node, group_config, group_prefix);

    configurations.push_back(group_config);
  }
  return configurations;
}

}  // namespace config_utils

#endif  // ROSBAG2_PERFORMANCE_BENCHMARKING__CONFIG_UTILS_HPP_
