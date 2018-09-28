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

#ifndef ROSBAG2__FORMATTER_HPP_
#define ROSBAG2__FORMATTER_HPP_

#include <chrono>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "rosbag2/types.hpp"

namespace rosbag2
{

class Formatter
{
public:
  std::map<std::string, std::string> format_duration(
    std::chrono::high_resolution_clock::duration duration);
  std::string format_time_point(std::chrono::high_resolution_clock::duration time_point);
  std::string format_file_size(size_t file_size);
  void format_file_paths(std::vector<std::string> paths, std::stringstream & info_stream);
  void format_topics_with_type(std::vector<TopicMetadata>, std::stringstream & info_stream);
};

}  // namespace rosbag2

#endif  // ROSBAG2__FORMATTER_HPP_
