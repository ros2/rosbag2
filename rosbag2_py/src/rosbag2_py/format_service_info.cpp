// Copyright 2023 Sony Group Corporation.
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

#include <vector>
#include <memory>
#include <sstream>
#include <string>

#include "format_service_info.hpp"
#include "rosbag2_cpp/service_utils.hpp"

namespace
{

std::string format_file_size(uint64_t file_size)
{
  double size = static_cast<double>(file_size);
  static const char * units[] = {"B", "KiB", "MiB", "GiB", "TiB"};
  double reference_number_bytes = 1024;
  int index = 0;
  while (size >= reference_number_bytes && index < 4) {
    size /= reference_number_bytes;
    index++;
  }

  std::stringstream rounded_size;
  int size_format_precision = index == 0 ? 0 : 1;
  rounded_size << std::setprecision(size_format_precision) << std::fixed << size;
  return rounded_size.str() + " " + units[index];
}

}  // namespace

namespace rosbag2_py
{

std::string
format_service_info(
  std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> & service_info_list,
  const std::unordered_map<std::string, uint64_t> & messages_size,
  bool verbose,
  const InfoSortingMethod sort_method)
{
  std::stringstream info_stream;
  const std::string service_info_string = "Service information: ";
  auto indentation_spaces = service_info_string.size();
  info_stream << "Service:           " << service_info_list.size() << std::endl;
  info_stream << service_info_string;

  if (service_info_list.empty()) {
    return info_stream.str();
  }

  auto print_service_info =
    [&info_stream, &messages_size, verbose](
    const std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t> & si) -> void {
      info_stream << "Service: " << si->name << " | ";
      info_stream << "Type: " << si->type << " | ";
      info_stream << "Request Count: " << si->request_count << " | ";
      info_stream << "Response Count: " << si->response_count << " | ";
      if (verbose) {
        uint64_t service_size = 0;
        auto service_size_iter = messages_size.find(
          rosbag2_cpp::service_name_to_service_event_topic_name(si->name));
        if (service_size_iter != messages_size.end()) {
          service_size = service_size_iter->second;
        }
        info_stream << "Size Contribution: " << format_file_size(service_size) << " | ";
      }
      info_stream << "Serialization Format: " << si->serialization_format;
      info_stream << std::endl;
    };

  std::vector<size_t> sorted_idx = generate_sorted_idx(service_info_list, sort_method);

  print_service_info(service_info_list[sorted_idx[0]]);
  auto number_of_services = service_info_list.size();
  for (size_t j = 1; j < number_of_services; ++j) {
    info_stream << std::string(indentation_spaces, ' ');
    print_service_info(service_info_list[sorted_idx[j]]);
  }

  return info_stream.str();
}

}  // namespace rosbag2_py
