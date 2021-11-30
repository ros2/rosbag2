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
#include <vector>
#include <unordered_map>
#include <utility>

#include "rcpputils/split.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"

#include "logging.hpp"
#include "topic_filter.hpp"

namespace
{
bool has_single_type(
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


bool topic_is_hidden(const std::string & topic_name)
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

bool topic_in_list(const std::string & topic_name, const std::vector<std::string> & topics)
{
  auto it = std::find(topics.begin(), topics.end(), topic_name);
  return it != topics.end();
}

}  // namespace

namespace rosbag2_transport
{

TopicFilter::TopicFilter(RecordOptions record_options, bool allow_unknown_types)
: record_options_(record_options),
  allow_unknown_types_(allow_unknown_types)
{}

TopicFilter::~TopicFilter()
{}

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
  if (!allow_unknown_types_ && !type_is_known(topic_name, topic_type)) {
    return false;
  }

  if (!record_options_.include_hidden_topics && topic_is_hidden(topic_name)) {
    ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
      "Hidden topics are not recorded. Enable them with --include-hidden-topics");
    return false;
  }

  if (!record_options_.topics.empty() && !topic_in_list(topic_name, record_options_.topics)) {
    return false;
  }

  std::regex exclude_regex(record_options_.exclude);
  if (std::regex_match(topic_name, exclude_regex)) {
    return false;
  }

  std::regex include_regex(record_options_.regex);
  if (
    !record_options_.all &&  // All takes precedence over regex
    !record_options_.regex.empty() &&  // empty regex matches nothing, but should be ignored
    !std::regex_match(topic_name, include_regex))
  {
    return false;
  }

  return true;
}

bool TopicFilter::type_is_known(const std::string & topic_name, const std::string & topic_type)
{
  try {
    auto package_name = std::get<0>(rosbag2_cpp::extract_type_identifier(topic_type));
    rosbag2_cpp::get_typesupport_library_path(package_name, "rosidl_typesupport_cpp");
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
