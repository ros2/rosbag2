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
#include <utility>
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

  // Expose getters for caches for testing purposes
  const std::unordered_map<std::string, std::pair<bool, bool>> & get_take_topics_cache() const
  {
    return take_topics_cache_;
  }

  const std::unordered_set<std::string> & get_unknown_types_cache() const
  {
    return unknown_types_cache_;
  }

  bool get_allow_unknown_types() const
  {
    return allow_unknown_types_;
  }

  // Add setter for cache entries to support testing cache behavior
  void set_topic_cache_entry(const std::string & topic_name, bool take_topic, bool type_unknown)
  {
    take_topics_cache_[topic_name] = {take_topic, type_unknown};
  }

  // Clear the cache for testing purposes
  void clear_caches()
  {
    take_topics_cache_.clear();
    unknown_types_cache_.clear();
  }

private:
  /// Return true if the topic passes all filter criteria
  bool take_topic(const std::string & topic_name, const std::vector<std::string> & topic_types);
  bool type_is_known(const std::string & topic_name, const std::string & topic_type);

  RecordOptions record_options_;
  bool allow_unknown_types_ = false;
  std::unordered_set<std::string> unknown_types_cache_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;

  /// The action name in record_options.include_action will be converted into the action interface
  ///  name and saved in this set
  std::unordered_set<std::string> include_action_interface_names_;

  /// The action name in record_options.exclude_action will be converted into the action interface
  ///  name and saved in this set
  std::unordered_set<std::string> exclude_action_interface_names_;

  /// Cache for already filtered topics. The pair contains:
  /// - first: take topic result
  /// - second: whether the topic type is unknown when the topic was checked
  /// This is to avoid checking the same unknown type again and again.
  /// Note: This cache does not guarantee to be always correct, because the topic type may
  /// change over time. But it is good enough for performance consideration.
  std::unordered_map<std::string, std::pair<bool, bool>> take_topics_cache_;
};
}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__TOPIC_FILTER_HPP_
