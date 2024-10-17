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

#include "rosbag2_storage/bag_metadata.hpp"

#include "info_sorting_method.hpp"


namespace rosbag2_py
{

InfoSortingMethod info_sorting_method_from_string(std::string str)
{
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  auto find_result = sorting_method_map.find(str);
  if (find_result == sorting_method_map.end()) {
    throw std::runtime_error("Enum value match for \"" + str + "\" string is not found.");
  }
  return find_result->second;
}

std::vector<size_t> generate_sorted_idx(
  const std::vector<rosbag2_storage::TopicInformation> & topics,
  const InfoSortingMethod sort_method)
{
  std::vector<size_t> sorted_idx(topics.size());
  std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
  std::sort(
    sorted_idx.begin(),
    sorted_idx.end(),
    [&topics, sort_method](size_t i1, size_t i2) {
      bool is_greater = false;
      switch (sort_method) {
        case InfoSortingMethod::NAME:
          is_greater = topics[i1].topic_metadata.name < topics[i2].topic_metadata.name;
          break;
        case InfoSortingMethod::TYPE:
          is_greater = topics[i1].topic_metadata.type < topics[i2].topic_metadata.type;
          break;
        case InfoSortingMethod::COUNT:
          is_greater = topics[i1].message_count < topics[i2].message_count;
          break;
        default:
          throw std::runtime_error("switch is not exhaustive");
      }
      return is_greater;
    }
  );
  return sorted_idx;
}


std::vector<size_t> generate_sorted_idx(
  const std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> & services,
  const InfoSortingMethod sort_method)
{
  std::vector<size_t> sorted_idx(services.size());
  std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
  std::sort(
    sorted_idx.begin(),
    sorted_idx.end(),
    [&services, sort_method](size_t i1, size_t i2) {
      bool is_greater = false;
      switch (sort_method) {
        case InfoSortingMethod::NAME:
          is_greater = services[i1]->name < services[i2]->name;
          break;
        case InfoSortingMethod::TYPE:
          is_greater = services[i1]->type < services[i2]->type;
          break;
        case InfoSortingMethod::COUNT:
          {
            const auto & count_1 = services[i1]->request_count + services[i1]->response_count;
            const auto & count_2 = services[i2]->request_count + services[i2]->response_count;
            is_greater = count_1 < count_2;
            break;
          }
        default:
          throw std::runtime_error("switch is not exhaustive");
      }
      return is_greater;
    }
  );
  return sorted_idx;
}


std::vector<size_t> generate_sorted_idx(
  const std::vector<std::shared_ptr<ServiceEventInformation>> & services,
  const InfoSortingMethod sort_method)
{
  std::vector<size_t> sorted_idx(services.size());
  std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
  std::sort(
    sorted_idx.begin(),
    sorted_idx.end(),
    [&services, sort_method](size_t i1, size_t i2) {
      bool is_greater = false;
      switch (sort_method) {
        case InfoSortingMethod::NAME:
          is_greater = services[i1]->service_metadata.name < services[i2]->service_metadata.name;
          break;
        case InfoSortingMethod::TYPE:
          is_greater = services[i1]->service_metadata.type < services[i2]->service_metadata.type;
          break;
        case InfoSortingMethod::COUNT:
          is_greater = services[i1]->event_message_count < services[i2]->event_message_count;
          break;
        default:
          throw std::runtime_error("switch is not exhaustive");
      }
      return is_greater;
    }
  );
  return sorted_idx;
}

}  // namespace rosbag2_py
