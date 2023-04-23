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

#ifndef ROSBAG2_PERFORMANCE_BENCHMARKING__RESULT_UTILS_HPP_
#define ROSBAG2_PERFORMANCE_BENCHMARKING__RESULT_UTILS_HPP_

#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "rosbag2_performance_benchmarking/bag_config.hpp"
#include "rosbag2_performance_benchmarking/publisher_group_config.hpp"

namespace result_utils
{

/// Read total count of recorded messages from metadata.yaml file
int get_message_count_from_metadata(const std::string & uri);

/// Based on configuration and metadata from completed benchmark, write results
void write_benchmark_results(
  const std::vector<PublisherGroupConfig> & publisher_groups_config,
  const BagConfig & bag_config,
  const std::string & results_file,
  float producer_cpu_usage = 0,
  float recorder_cpu_usage = 0,
  const std::vector<double> & cpu_usage_per_core = {});

/// this version works with a standalone node using node parameters
void write_benchmark_results(rclcpp::Node & node);

}  // namespace result_utils

#endif  // ROSBAG2_PERFORMANCE_BENCHMARKING__RESULT_UTILS_HPP_
