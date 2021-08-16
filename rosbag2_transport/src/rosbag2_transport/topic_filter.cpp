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
#include <unordered_set>

#include "rclcpp/logging.hpp"

#include "rcpputils/split.hpp"

#include "./topic_filter.hpp"

namespace rosbag2_transport
{
namespace topic_filter
{

std::unordered_map<std::string, std::string> filter_topics(
  const std::vector<std::string> & selected_topic_names,
  const std::unordered_map<std::string, std::string> & all_topic_names_and_types)
{
  std::unordered_map<std::string, std::string> filtered_topics_and_types;

  auto topic_name_matches = [&selected_topic_names](const auto & topic_and_type) -> bool
    {
      return std::find(
        selected_topic_names.begin(),
        selected_topic_names.end(), topic_and_type.first) != selected_topic_names.end();
    };

  for (const auto & topic_and_type : all_topic_names_and_types) {
    if (topic_name_matches(topic_and_type)) {
      filtered_topics_and_types.insert(topic_and_type);
    }
  }

  return filtered_topics_and_types;
}

std::unordered_map<std::string, std::string> filter_topics_with_more_than_one_type(
  const std::map<std::string, std::vector<std::string>> & topics_and_types,
  bool include_hidden_topics)
{
  std::unordered_map<std::string, std::string> filtered_topics_and_types;

  auto logger = rclcpp::get_logger("rosbag2_transport");

  for (const auto & topic_and_type : topics_and_types) {
    if (topic_and_type.second.size() > 1) {
      RCLCPP_ERROR_STREAM(
        logger,
        "Topic '" << topic_and_type.first <<
          "' has several types associated. Only topics with one type are supported");
      continue;
    }

    // According to rclpy's implementation, the indicator for a hidden topic is a leading '_'
    // https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/topic_or_service_is_hidden.py#L15
    if (!include_hidden_topics) {
      auto tokens = rcpputils::split(topic_and_type.first, '/', true);  // skip empty
      auto is_hidden = std::find_if(
        tokens.begin(), tokens.end(), [](const auto & token) -> bool {
          return token[0] == '_';
        });
      if (is_hidden != tokens.end()) {
        RCLCPP_WARN_ONCE(
          logger,
          "Hidden topics are not recorded. Enable them with --include-hidden-topics");
        continue;
      }
    }

    filtered_topics_and_types.insert({topic_and_type.first, topic_and_type.second[0]});
  }
  return filtered_topics_and_types;
}

std::unordered_map<std::string, std::string>
filter_topics_using_regex(
  const std::unordered_map<std::string, std::string> & topics_and_types,
  const std::string & filter_regex_string,
  const std::string & exclude_regex_string,
  bool all_flag
)
{
  std::unordered_map<std::string, std::string> filtered_by_regex;

  std::regex filter_regex(filter_regex_string);
  std::regex exclude_regex(exclude_regex_string);

  for (const auto & kv : topics_and_types) {
    bool take = all_flag;
    // regex_match returns false for 'empty' regex
    if (!all_flag && !filter_regex_string.empty()) {
      take = std::regex_match(kv.first, filter_regex);
    }
    if (take) {
      take = !std::regex_match(kv.first, exclude_regex);
    }
    if (take) {
      filtered_by_regex.insert(kv);
    }
  }
  return filtered_by_regex;
}

std::unordered_map<std::string, std::string>
filter_topics_with_known_type(
  const std::unordered_map<std::string, std::string> & topics_and_types,
  std::unordered_set<std::string> & topic_unknown_types)
{
  std::unordered_map<std::string, std::string> filtered_topics_and_types;

  for (const auto & topic_and_type : topics_and_types) {
    try {
      auto package_name = std::get<0>(rosbag2_cpp::extract_type_identifier(topic_and_type.second));
      rosbag2_cpp::get_typesupport_library_path(package_name, "rosidl_typesupport_cpp");
    } catch (std::runtime_error & e) {
      std::unordered_set<std::string>::const_iterator got = topic_unknown_types.find(
        topic_and_type.second);
      if (got == topic_unknown_types.end()) {
        topic_unknown_types.emplace(topic_and_type.second);
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger("rosbag2_transport"),
          "Topic '" << topic_and_type.first <<
            "' has unknown type '" << topic_and_type.second <<
            "' associated. Only topics with known type are supported. Reason: '" << e.what());
      }
      continue;
    }
    filtered_topics_and_types.insert(topic_and_type);
  }
  return filtered_topics_and_types;
}

}  // namespace topic_filter
}  // namespace rosbag2_transport
