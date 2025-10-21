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
  /// \brief Constructor
  /// @param record_options Options for filtering topics.
  /// @param node_graph Node graph interface, used to check if a topic is unpublished
  /// (i.e. has no publishers associated with topic) or a leaf topic (i.e. has no subscribers).
  /// If nullptr, unpublished and leaf topics cannot be filtered out and corresponding checks will
  /// be disabled.
  /// @param allow_unknown_types Allow unknown types, i.e. types for which type support cannot be
  /// loaded.
  explicit TopicFilter(
    RecordOptions record_options,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph = nullptr,
    bool allow_unknown_types = false);
  virtual ~TopicFilter();

  /// \brief Filter topics based on the options provided in the constructor
  /// \details Used to filter all topic_names_and_types received from the
  /// node->get_topic_names_and_types()
  /// Filtering order is:
  /// - remove topics with multiple types, unknown type, and hidden topics
  /// - topics list
  /// - exclude regex
  /// - include regex OR "all"
  /// - exclude topics list
  /// - topic types list
  /// - exclude topic types list
  /// - actions list
  /// - exclude actions list
  /// - services list
  /// - exclude services list
  /// - unpublished topics
  /// - leaf topics
  /// @param topic_names_and_types The map of topic names and their associated types to filter
  /// @return The filtered map of topic names and their associated types
  std::unordered_map<std::string, std::string> filter_topics(
    const std::map<std::string, std::vector<std::string>> & topic_names_and_types);

protected:
  /// \brief Check if the topic is selected by include/exclude lists or regexes
  /// @param topic_name - the name of the topic to check
  /// @param topic_type - the type of the topic to check
  /// @return Return true if the topic is selected, false otherwise
  bool topic_selected_by_lists_or_regex(
    const std::string & topic_name,
    const std::string & topic_type);

  /// \brief Check if the topic should be taken (i.e. recorded) based on all filter criteria
  /// @details The checks performed are:
  /// - selected by include/exclude lists or regexes
  /// - type is known (i.e. type support can be loaded)
  /// - not unpublished (i.e. has at least one publisher associated with topic)
  /// - not a leaf topic (i.e. has at least one subscriber associated with topic)
  /// @param topic_name - the name of the topic to check
  /// @param topic_types - the types of the topic to check
  /// @return Return true if the topic passes all filter criteria, false otherwise
  bool take_topic(const std::string & topic_name, const std::vector<std::string> & topic_types);

  /// \brief Check if the topic type is known (i.e. type support can be loaded)
  /// @param topic_name - the name of the topic to check
  /// @param topic_type - the type of the topic to check
  /// @return Return true if the topic type support can be loaded, false otherwise
  bool type_is_known(const std::string & topic_name, const std::string & topic_type);

  /// Cache for topic_selected_by_lists_or_regex results
  /// The key is a concatenation of topic name and topic type to avoid ambiguity
  /// \note Uses kTypeNameDelimiter_ as delimiter between topic name and topic type in the key
  std::unordered_map<std::string, bool> topic_selected_by_lists_or_regex_cache_;

  /// Delimiter used for concatenating topic name and topic type in the cache key
  /// Using a special character to avoid collisions with topic names and types. i.e. without
  /// delimiter topic name "/motor" and type "controlsystem_msgs/msg/Foo" would create the same
  /// key as a topic name "/motorcontrol" and type "system_msgs/msg/Foo". The '&' character is not
  /// allowed in ROS topic names and types, so it is safe to use as delimiter.
  static constexpr const char * kTopicNameTypeDelimiter_ = "&";

  /// Cache for type_is_known results
  std::unordered_map<std::string, bool> known_topic_types_cache_;

  /// Remember which unknown types have already been warned about to avoid spamming the console
  std::unordered_set<std::string> already_warned_unknown_types_;

private:
  /// Options for filtering topics
  RecordOptions record_options_;

  /// Allow unknown types, i.e. types for which type support cannot be loaded
  bool allow_unknown_types_ = false;

  /// Node graph interface, used to check if a topic is unpublished or a leaf topic
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;

  /// The action name in record_options.include_action will be converted into the action interface
  /// name and saved in this set
  std::unordered_set<std::string> include_action_interface_names_;

  /// The action name in record_options.exclude_action will be converted into the action interface
  /// name and saved in this set
  std::unordered_set<std::string> exclude_action_interface_names_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__TOPIC_FILTER_HPP_
