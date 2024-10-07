// Copyright 2024 Open Source Robotics Foundation, Inc.
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


#ifndef ROSBAG2_PY__INFO_SORTING_METHOD_HPP_
#define ROSBAG2_PY__INFO_SORTING_METHOD_HPP_

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>

#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_cpp/info.hpp"
#include "service_event_info.hpp"

namespace rosbag2_py
{

/// \brief Available sorting methods for info output.
enum class InfoSortingMethod
{
  NAME,
  TYPE,
  COUNT,
};

const std::unordered_map<std::string, InfoSortingMethod> sorting_method_map = {
  {"name", InfoSortingMethod::NAME},
  {"type", InfoSortingMethod::TYPE},
  {"count", InfoSortingMethod::COUNT}};

InfoSortingMethod info_sorting_method_from_string(std::string str);

std::vector<size_t> generate_sorted_idx(
  const std::vector<rosbag2_storage::TopicInformation> & topics,
  InfoSortingMethod sort_method = InfoSortingMethod::NAME);

std::vector<size_t> generate_sorted_idx(
  const std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> & services,
  InfoSortingMethod sort_method = InfoSortingMethod::NAME);

std::vector<size_t> generate_sorted_idx(
  const std::vector<std::shared_ptr<ServiceEventInformation>> & services,
  InfoSortingMethod sort_method = InfoSortingMethod::NAME);

}  // namespace rosbag2_py

#endif  // ROSBAG2_PY__INFO_SORTING_METHOD_HPP_
