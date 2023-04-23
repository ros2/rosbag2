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

#include "rosbag2_performance_benchmarking/result_utils.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>   // std::setprecision, std::setw
#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/yaml.hpp"

#include "rosbag2_performance_benchmarking/bag_config.hpp"
#include "rosbag2_performance_benchmarking/publisher_group_config.hpp"
#include "rosbag2_performance_benchmarking/config_utils.hpp"

namespace result_utils
{

/// Read total count of recorded messages from metadata.yaml file
int get_message_count_from_metadata(const std::string & uri)
{
  int total_recorded_count = 0;
  std::string metadata_filename(rosbag2_storage::MetadataIo::metadata_filename);
  std::string metadata_path = uri + "/" + metadata_filename;
  try {
    YAML::Node yaml_file = YAML::LoadFile(metadata_path);
    total_recorded_count = yaml_file["rosbag2_bagfile_information"]["message_count"].as<int>();
  } catch (const YAML::Exception & ex) {
    throw std::runtime_error(
            std::string("Exception on parsing metadata file to get total message count: ") +
            metadata_path + " " +
            ex.what());
  }
  return total_recorded_count;
}

/// Based on configuration and metadata from completed benchmark, write results
void write_benchmark_results(
  const std::vector<PublisherGroupConfig> & publisher_groups_config,
  const BagConfig & bag_config,
  const std::string & results_file,
  float producer_cpu_usage,
  float recorder_cpu_usage,
  const std::vector<double> & cpu_usage_per_core)
{
  bool new_file = false;
  { // test if file exists - we want to write a csv header after creation if not
    // use std::filesystem when switching to C++17
    std::ifstream test_existence(results_file);
    if (!test_existence) {
      new_file = true;
    }
  }

  // append, we want to accumulate results from multiple runs
  std::ofstream output_file(results_file, std::ios_base::app);
  if (!output_file.is_open()) {
    throw std::runtime_error(std::string("Could not open file: ") + results_file);
  }

  if (new_file) {
    output_file << "storage_id ";
    output_file << "instances frequency message_size total_messages_sent cache_size ";
    output_file << "max_bagfile_size storage_config ";
    output_file << "compression compression_queue compression_threads ";
    output_file << "total_produced total_recorded_count ";
    output_file << "producer_cpu_usage recorder_cpu_usage";
    for (size_t i = 0; i < cpu_usage_per_core.size(); i++) {
      output_file << " core_" << i;
    }
    output_file << std::endl;
  }

  int total_recorded_count = get_message_count_from_metadata(bag_config.storage_options.uri);

  for (const auto & c : publisher_groups_config) {
    output_file << bag_config.storage_options.storage_id << " ";
    output_file << c.count << " ";
    output_file << c.producer_config.frequency << " ";
    output_file << c.producer_config.message_size << " ";
    output_file << c.producer_config.max_count << " ";
    output_file << bag_config.storage_options.max_cache_size << " ";
    output_file << bag_config.storage_options.max_bagfile_size << " ";
    output_file << bag_config.storage_options.storage_config_uri << " ";
    output_file << bag_config.compression_format << " ";
    output_file << bag_config.compression_queue_size << " ";
    output_file << bag_config.compression_threads << " ";

    // TODO(adamdbrw) - this is a result for the entire group,
    // but we don't yet have per-group stats.
    // For now, these need to be summed for each group
    auto total_messages_produced = c.producer_config.max_count * c.count;
    output_file << total_messages_produced << " ";
    output_file << total_recorded_count << " ";
    output_file << std::fixed;               // Fix the number of decimal digits
    output_file << std::setprecision(2);  // to 2
    output_file << std::setw(4) << producer_cpu_usage << " ";
    output_file << std::setw(4) << recorder_cpu_usage;
    for (auto cpu_core_usage : cpu_usage_per_core) {
      output_file << " " << std::setw(4) << cpu_core_usage;
    }
    output_file << std::endl;
  }
}

/// this version works with a standalone node using node parameters
void write_benchmark_results(rclcpp::Node & node)
{
  auto configurations = config_utils::publisher_groups_from_node_parameters(node);
  auto bag_config = config_utils::bag_config_from_node_parameters(node);

  std::string results_file;
  node.declare_parameter("results_file", bag_config.storage_options.uri + "/results.csv");
  node.get_parameter("results_file", results_file);

  float recorder_cpu_usage = 0;
  node.declare_parameter("recorder_cpu_usage", 0.0);
  node.get_parameter("recorder_cpu_usage", recorder_cpu_usage);

  float producer_cpu_usage = 0;
  node.declare_parameter("producer_cpu_usage", 0.0);
  node.get_parameter("producer_cpu_usage", producer_cpu_usage);

  node.declare_parameter("cpu_usage_per_core", std::vector<double>{});
  auto cpu_usage_per_core = node.get_parameter("cpu_usage_per_core").as_double_array();

  write_benchmark_results(
    configurations, bag_config, results_file,
    producer_cpu_usage, recorder_cpu_usage, cpu_usage_per_core);
}

}  // namespace result_utils
