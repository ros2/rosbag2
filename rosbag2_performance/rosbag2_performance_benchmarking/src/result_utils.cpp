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
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/yaml.hpp"

#include "rosbag2_performance_benchmarking/bag_config.hpp"
#include "rosbag2_performance_benchmarking/publisher_group_config.hpp"
#include "rosbag2_performance_benchmarking/config_utils.hpp"

namespace result_utils
{

int get_message_count_from_metadata(const std::string & uri)
{
  rosbag2_storage::MetadataIo mdio;
  auto metadata = mdio.read_metadata(uri);
  return metadata.message_count;
}

int get_total_size_from_testdata(const std::string & uri)
{
  rosbag2_storage::MetadataIo mdio;
  auto metadata = mdio.read_metadata(uri);
  int total_size = 0;
  for (const auto & fileinfo : metadata.files) {
    std::filesystem::path p(uri);
    p /= fileinfo.path;
    total_size += std::filesystem::file_size(p);
  }
  return total_size;
}

/// Based on configuration and metadata from completed benchmark, write results
void write_benchmark_results(
  const std::vector<PublisherGroupConfig> & publisher_groups_config,
  const BagConfig & bag_config,
  const std::string & results_file)
{
  const auto delim = ',';
  bool new_file = false;
  { // test if file exists - we want to write a csv header after creation if not
    // use std::filesystem when switching to C++17
    std::ifstream test_existence(results_file);
    if (!test_existence) {
      new_file = true;
    }
  }

  // append, we want to accumulate results from multiple runs
  std::ofstream of(results_file, std::ios_base::app);
  if (!of.is_open()) {
    throw std::runtime_error(std::string("Could not open file: ") + results_file);
  }

  if (new_file) {
    for (auto field : {
      // "instances", "frequency", "message_size", "total_messages_sent",
      "cache_size", "max_bagfile_size", "storage_id", "storage_config", "compression",
      "compression_queue", "compression_threads",
      "total_produced", "total_recorded_count", "drop_ratio",
      "total_produced_size", "total_recorded_size", "disk_overhead"}
    ) {
      of << field << delim;
    }
    of << std::endl;
  }

  int total_size = get_total_size_from_testdata(bag_config.storage_options.uri);
  int total_recorded_count = get_message_count_from_metadata(bag_config.storage_options.uri);
  int total_messages_produced = 0;
  int total_expected_datasize = 0;
  for (const auto & c : publisher_groups_config) {
    auto messages_of_this_kind = c.producer_config.max_count * c.count;
    total_messages_produced += messages_of_this_kind;
    total_expected_datasize += messages_of_this_kind * c.producer_config.message_size;
  }

  auto dropped = total_messages_produced - total_recorded_count;
  double drop_ratio = dropped / double(total_messages_produced);

  // Test params
  of << bag_config.storage_options.max_cache_size << delim;
  of << bag_config.storage_options.max_bagfile_size << delim;
  of << bag_config.storage_options.storage_id << delim;
  of << bag_config.storage_options.storage_config_uri << delim;
  of << bag_config.compression_format << delim;
  of << bag_config.compression_queue_size << delim;
  of << bag_config.compression_threads << delim;

  // Result counts
  of << total_messages_produced << delim;
  of << total_recorded_count << delim;
  of << drop_ratio << delim;

  // Result sizes
  of << total_expected_datasize << delim;
  of << total_size << delim;
  auto overhead = total_size - total_expected_datasize;
  of << overhead << delim;

  of << std::endl;
}

/// this version works with a standalone node using node parameters
void write_benchmark_results(rclcpp::Node & node)
{
  auto configurations = config_utils::publisher_groups_from_node_parameters(node);
  auto bag_config = config_utils::bag_config_from_node_parameters(node);

  std::string results_file;
  node.declare_parameter("results_file", bag_config.storage_options.uri + "/results.csv");
  node.get_parameter("results_file", results_file);

  write_benchmark_results(configurations, bag_config, results_file);
}

}  // namespace result_utils
