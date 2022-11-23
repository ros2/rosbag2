// Copyright 2021, Robotec.ai sp. z o.o.
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

#include "rosbag2_performance_benchmarking/config_utils.hpp"

#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rosbag2_performance_benchmarking/producer_config.hpp"
#include "rosbag2_storage/default_storage_id.hpp"

namespace config_utils
{

void load_qos_configuration(
  rclcpp::Node & node,
  PublisherGroupConfig & group_config,
  const std::string & group_prefix)
{
  unsigned int qos_depth = 10;
  std::string qos_reliability = "reliable";
  std::string qos_durability = "volatile";
  auto qos_prefix = group_prefix + ".qos";
  node.declare_parameter<int>(qos_prefix + ".qos_depth", qos_depth);
  node.declare_parameter<std::string>(qos_prefix + ".qos_reliability", qos_reliability);
  node.declare_parameter<std::string>(qos_prefix + ".qos_durability", qos_durability);

  node.get_parameter(qos_prefix + ".qos_depth", qos_depth);
  node.get_parameter(qos_prefix + ".qos_reliability", qos_reliability);
  node.get_parameter(qos_prefix + ".qos_durability", qos_durability);

  group_config.qos.keep_last(qos_depth);
  // TODO(adamdbrw) - error handling / map string to function
  if (qos_reliability == "reliable") {group_config.qos.reliable();}
  if (qos_reliability == "best_effort") {group_config.qos.best_effort();}
  if (qos_durability == "transient_local") {group_config.qos.transient_local();}
  if (qos_durability == "volatile") {group_config.qos.durability_volatile();}
}

bool wait_for_subscriptions_from_node_parameters(rclcpp::Node & node)
{
  const std::string parameters_ns = "publishers";
  node.declare_parameter<bool>(parameters_ns + ".wait_for_subscriptions", true);
  bool wait_for_subscriptions;
  node.get_parameter(parameters_ns + ".wait_for_subscriptions", wait_for_subscriptions);
  return wait_for_subscriptions;
}

std::vector<PublisherGroupConfig> publisher_groups_from_node_parameters(
  rclcpp::Node & node)
{
  std::vector<PublisherGroupConfig> configurations;
  std::vector<std::string> publisher_groups;
  const std::string parameters_ns = "publishers";
  node.declare_parameter<std::vector<std::string>>(parameters_ns + ".publisher_groups");
  node.get_parameter(parameters_ns + ".publisher_groups", publisher_groups);
  for (const auto & group_name : publisher_groups) {
    auto group_prefix = parameters_ns + "." + group_name;
    node.declare_parameter<int>(group_prefix + ".publishers_count");
    node.declare_parameter<std::string>(group_prefix + ".topic_root");
    node.declare_parameter<int>(group_prefix + ".msg_size_bytes");
    node.declare_parameter<int>(group_prefix + ".msg_count_each");
    node.declare_parameter<int>(group_prefix + ".rate_hz");

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

    if (group_config.producer_config.frequency == 0) {
      RCLCPP_ERROR(node.get_logger(), "Frequency can't be 0. Exiting.");
      rclcpp::shutdown(nullptr, "Invalid frequency parameter");
      return configurations;
    }

    config_utils::load_qos_configuration(node, group_config, group_prefix);

    configurations.push_back(group_config);
  }
  return configurations;
}

BagConfig bag_config_from_node_parameters(
  rclcpp::Node & node)
{
  const std::string default_bag_folder("/tmp/rosbag2_test");
  BagConfig bag_config;

  node.declare_parameter<std::string>("storage_id", rosbag2_storage::get_default_storage_id());
  node.declare_parameter<int>("max_cache_size", 10000000);
  node.declare_parameter<int>("max_bag_size", 0);
  node.declare_parameter<std::string>("db_folder", default_bag_folder);
  node.declare_parameter<std::string>("storage_config_file", "");
  node.declare_parameter<std::string>("compression_format", "");
  node.declare_parameter<int>("compression_queue_size", 1);
  node.declare_parameter<int>("compression_threads", 0);

  node.get_parameter("storage_id", bag_config.storage_options.storage_id);
  node.get_parameter("max_cache_size", bag_config.storage_options.max_cache_size);
  node.get_parameter("max_bag_size", bag_config.storage_options.max_bagfile_size);
  node.get_parameter("db_folder", bag_config.storage_options.uri);
  node.get_parameter("storage_config_file", bag_config.storage_options.storage_config_uri);
  node.get_parameter("compression_format", bag_config.compression_format);
  node.get_parameter("compression_queue_size", bag_config.compression_queue_size);
  node.get_parameter("compression_threads", bag_config.compression_threads);

  return bag_config;
}

}  // namespace config_utils
