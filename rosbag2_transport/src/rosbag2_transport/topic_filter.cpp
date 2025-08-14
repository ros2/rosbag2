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

#include <algorithm>
#include <map>
#include <regex>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rcpputils/split.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/service_utils.hpp"

#include "logging.hpp"
#include "rosbag2_transport/topic_filter.hpp"

namespace
{
inline bool has_single_type(
  const std::string & topic_name, const std::vector<std::string> & topic_types)
{
  if (topic_types.empty()) {
    ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
      "Topic " << topic_name << " has no associated types. "
        "This case shouldn't occur.");
    return false;
  }
  auto it = topic_types.begin();
  const std::string & reference_type = *it;
  for (; it != topic_types.end(); it++) {
    if (reference_type != *it) {
      ROSBAG2_TRANSPORT_LOG_ERROR_STREAM(
        "Topic '" << topic_name <<
          "' has more than one type associated. Only topics with one type are supported");
      return false;
    }
  }
  return true;
}


inline bool topic_is_hidden(const std::string & topic_name)
{
  // According to rclpy's implementation, the indicator for a hidden topic is a leading '_'
  // https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/topic_or_service_is_hidden.py#L15
  auto tokens = rcpputils::split(topic_name, '/', true);  // skip empty
  auto hidden_it = std::find_if(
    tokens.begin(), tokens.end(), [](const auto & token) -> bool {
      return token[0] == '_';
    });
  return hidden_it != tokens.end();
}

inline bool topic_in_list(const std::string & topic_name, const std::vector<std::string> & topics)
{
  auto it = std::find(topics.begin(), topics.end(), topic_name);
  return it != topics.end();
}

inline
bool topic_type_in_list(const std::string & type_name, const std::vector<std::string> & topic_types)
{
  auto it = std::find(topic_types.begin(), topic_types.end(), type_name);
  return it != topic_types.end();
}

inline bool topic_is_unpublished(
  const std::string & topic_name, rclcpp::node_interfaces::NodeGraphInterface & node_graph)
{
  auto publishers_info = node_graph.get_publishers_info_by_topic(topic_name);
  return publishers_info.empty();
}

inline bool is_leaf_topic(
  const std::string & topic_name, rclcpp::node_interfaces::NodeGraphInterface & node_graph)
{
  auto subscriptions_info = node_graph.get_subscriptions_info_by_topic(topic_name);
  return subscriptions_info.empty();
}
}  // namespace

namespace rosbag2_transport
{

TopicFilter::TopicFilter(
  RecordOptions record_options,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  bool allow_unknown_types)
: record_options_(std::move(record_options)),
  allow_unknown_types_(allow_unknown_types),
  node_graph_(node_graph)
{}

TopicFilter::~TopicFilter() = default;

std::unordered_map<std::string, std::string> TopicFilter::filter_topics(
  const std::map<std::string, std::vector<std::string>> & topic_names_and_types)
{
  std::unordered_map<std::string, std::string> filtered_topics;
  for (const auto & [topic_name, topic_types] : topic_names_and_types) {
    if (take_topic(topic_name, topic_types)) {
      filtered_topics.insert(std::make_pair(topic_name, topic_types[0]));
    }
  }
  return filtered_topics;
}

bool TopicFilter::take_topic(
  const std::string & topic_name, const std::vector<std::string> & topic_types)
{
  if (!has_single_type(topic_name, topic_types)) {
    return false;
  }

  const std::string & topic_type = topic_types[0];
  bool is_service_event_topic = rosbag2_cpp::is_service_event_topic(topic_name, topic_type);


  if (!is_service_event_topic) {
    if (!record_options_.all_topics &&
      record_options_.topics.empty() &&
      record_options_.topic_types.empty() &&
      record_options_.regex.empty() &&
      !record_options_.include_hidden_topics)
    {
      // Note: This check is needed to avoid extra checks in case if only services (not topics)
      // needs to be selected.
      return false;
    }

    if (!record_options_.all_topics) {
      // Not in include topic list. Note: all_topics shall override include topic lists
      if (!topic_in_list(topic_name, record_options_.topics) &&
        !topic_type_in_list(topic_type, record_options_.topic_types))
      {
        // Not match include regex
        if (!record_options_.regex.empty()) {
          std::regex include_regex(record_options_.regex);
          if (!std::regex_search(topic_name, include_regex)) {
            return false;
          }
        } else {
          return false;
        }
      }
    }

    if (topic_type_in_list(topic_type, record_options_.exclude_topic_types)) {
      return false;
    }

    if (topic_in_list(topic_name, record_options_.exclude_topics)) {
      return false;
    }

    if (!record_options_.exclude_regex.empty()) {
      std::regex exclude_regex(record_options_.exclude_regex);
      if (std::regex_search(topic_name, exclude_regex)) {
        return false;
      }
    }

    if (!record_options_.include_hidden_topics && topic_is_hidden(topic_name)) {
      RCUTILS_LOG_WARN_ONCE_NAMED(
        ROSBAG2_TRANSPORT_PACKAGE_NAME,
        "Hidden topics are not recorded. Enable them with --include-hidden-topics");
      return false;
    }
  } else {
    if (!record_options_.all_services &&
      record_options_.services.empty() &&
      record_options_.regex.empty())
    {
      // Note: This check is needed to avoid extra checks and service name conversion in case
      // if only topics (not services) needs to be selected.
      return false;
    }

    // Convert service event topic name to service name
    auto service_name = rosbag2_cpp::service_event_topic_name_to_service_name(topic_name);

    if (!record_options_.all_services) {
      // Not in include service list
      if (!topic_in_list(topic_name, record_options_.services)) {
        // Not match include regex
        if (!record_options_.regex.empty()) {
          std::regex include_regex(record_options_.regex);
          if (!std::regex_search(service_name, include_regex)) {
            return false;
          }
        } else {
          return false;
        }
      }
    }

    if (topic_in_list(topic_name, record_options_.exclude_service_events)) {
      return false;
    }

    if (!record_options_.exclude_regex.empty()) {
      std::regex exclude_regex(record_options_.exclude_regex);
      if (std::regex_search(service_name, exclude_regex)) {
        return false;
      }
    }
  }

  if (!allow_unknown_types_ && !type_is_known(topic_name, topic_type)) {
    return false;
  }

  if (!record_options_.include_unpublished_topics && node_graph_ &&
    topic_is_unpublished(topic_name, *node_graph_))
  {
    return false;
  }

  if (record_options_.ignore_leaf_topics && node_graph_ &&
    is_leaf_topic(topic_name, *node_graph_))
  {
    return false;
  }

  return true;
}

bool TopicFilter::type_is_known(const std::string & topic_name, const std::string & topic_type)
{
  try {
    auto package_name = std::get<0>(rclcpp::extract_type_identifier(topic_type));
    rclcpp::get_typesupport_library_path(package_name, "rosidl_typesupport_cpp");
  } catch (std::runtime_error & e) {
    if (already_warned_unknown_types_.find(topic_type) == already_warned_unknown_types_.end()) {
      already_warned_unknown_types_.emplace(topic_type);
      ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
        "Topic '" << topic_name <<
          "' has unknown type '" << topic_type <<
          "' . Only topics with known type are supported. Reason: '" << e.what());
    }
    return false;
  }
  return true;
}
}  // namespace rosbag2_transport
