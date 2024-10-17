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

#ifndef ROSBAG2_PY__FORMAT_SERVICE_INFO_HPP_
#define ROSBAG2_PY__FORMAT_SERVICE_INFO_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "info_sorting_method.hpp"
#include "rosbag2_cpp/info.hpp"

namespace rosbag2_py
{

std::string format_service_info(
  std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> & service_info,
  const std::unordered_map<std::string, uint64_t> & messages_size = {},
  bool verbose = false,
  const InfoSortingMethod sort_method = InfoSortingMethod::NAME);

}  // namespace rosbag2_py

#endif  // ROSBAG2_PY__FORMAT_SERVICE_INFO_HPP_
