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

#ifndef ROSBAG2_CPP__INFO_HPP_
#define ROSBAG2_CPP__INFO_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "rosbag2_cpp/visibility_control.hpp"

#include "rosbag2_storage/bag_metadata.hpp"

namespace rosbag2_cpp
{

typedef ROSBAG2_CPP_PUBLIC_TYPE struct rosbag2_service_info_t
{
  std::string name;
  std::string type;
  std::string serialization_format;
  size_t request_count;
  size_t response_count;
} rosbag2_service_info_t;

class ROSBAG2_CPP_PUBLIC Info
{
public:
  virtual ~Info() = default;

  virtual rosbag2_storage::BagMetadata read_metadata(
    const std::string & uri, const std::string & storage_id = "");

  virtual std::vector<std::shared_ptr<rosbag2_service_info_t>> read_service_info(
    const std::string & uri, const std::string & storage_id = "");

  std::unordered_map<std::string, uint64_t> compute_messages_size_contribution(
    const std::string & uri, const std::string & storage_id = "");
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__INFO_HPP_
