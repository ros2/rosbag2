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

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_transport
{

class ROSBAG2_TRANSPORT_PUBLIC TopicFilter
{
public:
  explicit TopicFilter(
    RecordOptions record_options,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph = nullptr,
    bool allow_unknown_types = false);
  virtual ~TopicFilter();

  /// Filter all topic_names_and_types via take_topic method, return the resulting filtered set
  /// Filtering order is:
  /// - remove topics with multiple types, unknown type, and hidden topics
  /// - topics list
  /// - exclude regex
  /// - include regex OR "all"
  std::unordered_map<std::string, std::string> filter_topics(
    const std::map<std::string, std::vector<std::string>> & topic_names_and_types);

private:
  /// Return true if the topic passes all filter criteria
  bool take_topic(const std::string & topic_name, const std::vector<std::string> & topic_types);
  bool type_is_known(const std::string & topic_name, const std::string & topic_type);

  RecordOptions record_options_;
  bool allow_unknown_types_ = false;
  std::unordered_set<std::string> already_warned_unknown_types_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
};
}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__TOPIC_FILTER_HPP_
