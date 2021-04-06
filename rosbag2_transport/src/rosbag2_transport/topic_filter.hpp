// Copyright 2021, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_TRANSPORT__TOPIC_FILTER_HPP_
#define ROSBAG2_TRANSPORT__TOPIC_FILTER_HPP_

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace rosbag2_transport
{
namespace topic_filter
{

std::unordered_map<std::string, std::string>
filter_topics(
  const std::vector<std::string> & selected_topic_names,
  const std::unordered_map<std::string, std::string> & all_topic_names_and_types);

std::unordered_map<std::string, std::string>
filter_topics_with_more_than_one_type(
  const std::map<std::string, std::vector<std::string>> & topics_and_types,
  bool include_hidden_topics = false);

}  // namespace topic_filter
}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__TOPIC_FILTER_HPP_
