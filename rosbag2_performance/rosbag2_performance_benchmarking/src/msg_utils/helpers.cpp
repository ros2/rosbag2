// Copyright 2022 Apex.AI, Inc.
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

#include "msg_utils/helpers.hpp"

#include <cstdlib>

namespace msg_utils
{
namespace helpers
{
void generate_data(rosbag2_performance_benchmarking_msgs::msg::ByteArray & array, size_t size)
{
  array.data.reserve(size);
  for (auto i = 0u; i < size; ++i) {
    array.data.emplace_back(std::rand() % 255);
  }
}

void generate_data(sensor_msgs::msg::Image & msg, size_t size)
{
  // TODO(carlossvg): calculate message base size to set total size
  msg.data.reserve(size);
  for (auto i = 0u; i < size; ++i) {
    msg.data.emplace_back(std::rand() % 255);
  }
}

void generate_data(sensor_msgs::msg::PointCloud2 & msg, size_t size)
{
  // TODO(carlossvg): calculate message base size to set total size
  msg.data.reserve(size);
  for (auto i = 0u; i < size; ++i) {
    msg.data.emplace_back(std::rand() % 255);
  }
}

}  // namespace helpers
}  // namespace msg_utils
