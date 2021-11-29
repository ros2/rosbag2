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
#include <unordered_set>
#include <vector>

#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{

class ROSBAG2_TRANSPORT_PUBLIC TopicFilter
{
public:
  explicit TopicFilter(RecordOptions record_options, bool allow_unknown_types = false);

  /// Filter all topic_names_and_types via take_topic method, return the resulting filtered set
  /// Filtering order is:
  /// - remove topics with more than one type or unknown type (unable to load typesupport)
  /// - RecordOptions::all xor RecordOptions::topics (topics takes precedence if both provided)
  /// - apply include/exclude regex if specified
  std::unordered_map<std::string, std::string> filter_topics(
    const std::map<std::string, std::vector<std::string>> & topic_names_and_types);

private:
  /// Return true if the topic passes all filter criteria
  bool take_topic(const std::string & topic_name, const std::vector<std::string> & topic_types);
  bool type_is_known(const std::string & topic_name, const std::string & topic_type);

  RecordOptions record_options_;
  bool allow_unknown_types_ = false;
  std::unordered_set<std::string> already_warned_unknown_types_;
};
}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__TOPIC_FILTER_HPP_
