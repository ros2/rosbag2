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

#ifndef ROSBAG2_STORAGE__BAG_METADATA_HPP_
#define ROSBAG2_STORAGE__BAG_METADATA_HPP_

#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <utility>

#include "rosbag2_storage/topic_with_type.hpp"

namespace rosbag2_storage
{

struct TopicMetadata
{
  TopicWithType topic_with_type;
  size_t message_count;
};

struct BagMetadata
{
  int version = 1;  // upgrade this number when changing the content of the struct
  size_t bag_size = 0;  // Will not be serialized
  std::string storage_identifier;
  std::string serialization_format;
  std::vector<std::string> relative_file_paths;
  std::chrono::nanoseconds duration;
  std::chrono::time_point<std::chrono::high_resolution_clock> starting_time;
  size_t message_count;
  std::vector<TopicMetadata> topics_with_message_count;
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__BAG_METADATA_HPP_
