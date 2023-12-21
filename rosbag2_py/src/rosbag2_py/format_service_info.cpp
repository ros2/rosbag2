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

#include <sstream>

#include "format_service_info.hpp"

namespace rosbag2_py
{

std::string
format_service_info(
  std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> & service_info_list)
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
    [&info_stream](const std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t> & si) -> void {
      info_stream << "Service: " << si->name << " | ";
      info_stream << "Type: " << si->type << " | ";
      info_stream << "Request Count: " << si->request_count << " | ";
      info_stream << "Response Count: " << si->response_count << " | ";
      info_stream << "Serialization Format: " << si->serialization_format;
      info_stream << std::endl;
    };

  print_service_info(service_info_list[0]);
  auto number_of_services = service_info_list.size();
  for (size_t j = 1; j < number_of_services; ++j) {
    info_stream << std::string(indentation_spaces, ' ');
    print_service_info(service_info_list[j]);
  }

  return info_stream.str();
}

}  // namespace rosbag2_py
