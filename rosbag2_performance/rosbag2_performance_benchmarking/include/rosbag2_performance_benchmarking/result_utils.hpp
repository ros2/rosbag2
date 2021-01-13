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

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/metadata_io.hpp"

#include "rosbag2_performance_benchmarking/bag_config.hpp"
#include "rosbag2_performance_benchmarking/publisher_group_config.hpp"

#ifdef _WIN32
// This is necessary because of a bug in yaml-cpp's cmake
#define YAML_CPP_DLL
// This is necessary because yaml-cpp does not always use dllimport/dllexport consistently
# pragma warning(push)
# pragma warning(disable:4251)
# pragma warning(disable:4275)
#endif
#include "yaml-cpp/yaml.h"
#ifdef _WIN32
# pragma warning(pop)
#endif

namespace result_utils
{

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

void write_benchmark_results(
  const std::vector<PublisherGroupConfig> & publisher_groups_config,
  const BagConfig & bag_config,
  const std::string & results_file)
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
    output_file << "instances frequency message_size total_messages_sent cache_size ";
    output_file << "max_bagfile_size storage_config ";
    output_file << "compression compression_queue compression_threads ";
    output_file << "total_produced total_recorded_count\n";
  }

  int total_recorded_count = get_message_count_from_metadata(bag_config.storage_options.uri);

  for (const auto & c : publisher_groups_config) {
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
    output_file << total_recorded_count << std::endl;
  }
}

}  // namespace result_utils

#endif  // ROSBAG2_PERFORMANCE_BENCHMARKING__RESULT_UTILS_HPP_
