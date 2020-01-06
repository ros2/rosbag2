// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_TRANSPORT__FORMATTER_HPP_
#define ROSBAG2_TRANSPORT__FORMATTER_HPP_

#include <chrono>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosbag2_storage/bag_metadata.hpp"

namespace rosbag2_transport
{

class Formatter
{
public:
  static void format_bag_meta_data(const rosbag2_storage::BagMetadata & metadata);

  static std::unordered_map<std::string, std::string> format_duration(
    std::chrono::high_resolution_clock::duration duration);

  static std::string format_time_point(std::chrono::high_resolution_clock::duration time_point);

  static std::string format_file_size(uint64_t file_size);

  static void format_file_paths(
    const std::vector<std::string> & paths,
    std::stringstream & info_stream,
    int indentation_spaces);

  static void format_topics_with_type(
    const std::vector<rosbag2_storage::TopicInformation> & topics,
    std::stringstream & info_stream,
    int indentation_spaces);

private:
  static void indent(std::stringstream & info_stream, int number_of_spaces);
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__FORMATTER_HPP_
