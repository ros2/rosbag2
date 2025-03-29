// Copyright 2025 Sony Group Corporation.
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

#include <cstring>
#include <string>
#include <regex>
#include <unordered_map>

#include "rosbag2_cpp/action_utils.hpp"

namespace rosbag2_cpp
{
// The postfix of the action internal topics and service event topics
const std::unordered_map<ActionInterfaceType, std::string> action_topic_to_postfix_map = {
  {ActionInterfaceType::SendGoalEvent, "/_action/send_goal/_service_event"},
  {ActionInterfaceType::CancelGoalEvent, "/_action/cancel_goal/_service_event"},
  {ActionInterfaceType::GetResultEvent, "/_action/get_result/_service_event"},
  {ActionInterfaceType::Feedback, "/_action/feedback"},
  {ActionInterfaceType::Status, "/_action/status"}
};

// The regex pattern of the action internal topics and service event topics
const std::unordered_map<ActionInterfaceType, std::string> action_topic_type_to_regex_map = {
  {ActionInterfaceType::SendGoalEvent, ".+/action/.+SendGoal_Event$"},
  {ActionInterfaceType::CancelGoalEvent, "^action_msgs/srv/CancelGoal_Event$"},
  {ActionInterfaceType::GetResultEvent, ".+/action/.+GetResult_Event$"},
  {ActionInterfaceType::Feedback, ".+/action/.+_FeedbackMessage$"},
  {ActionInterfaceType::Status, "^action_msgs/msg/GoalStatusArray$"}
};

const size_t kMinActionTopicPostfixLen =
  action_topic_to_postfix_map.at(ActionInterfaceType::Status).length();

bool is_topic_belong_to_action(const std::string & topic_name, const std::string & topic_type)
{
  ActionInterfaceType topic = get_action_interface_type(topic_name);
  if (topic == ActionInterfaceType::Unknown) {
    return false;
  }

  std::regex pattern(action_topic_type_to_regex_map.at(topic));
  return std::regex_search(topic_type, pattern);
}

bool is_topic_belong_to_action(
  ActionInterfaceType action_interface_type, const std::string & topic_type)
{
  if (action_interface_type == ActionInterfaceType::Unknown) {
    return false;
  }
  std::regex pattern(action_topic_type_to_regex_map.at(action_interface_type));
  return std::regex_search(topic_type, pattern);
}

std::string action_interface_name_to_action_name(const std::string & topic_name)
{
  std::string action_name;
  if (topic_name.length() <= kMinActionTopicPostfixLen) {
    return action_name;
  } else {
    for (const auto & [topic_type_enum, postfix] : action_topic_to_postfix_map) {
      if (topic_name.length() > postfix.length() &&
        topic_name.compare(topic_name.length() - postfix.length(), postfix.length(), postfix) == 0)
      {
        action_name = topic_name.substr(0, topic_name.length() - postfix.length());
        break;
      }
    }
  }

  return action_name;
}

std::string get_action_type_for_info(const std::string & topic_type)
{
  std::string action_type;

  for (auto &[topic_type_enum, regex] : action_topic_type_to_regex_map) {
    std::regex pattern(regex);
    if (std::regex_search(topic_type, pattern)) {
      switch (topic_type_enum) {
        case ActionInterfaceType::SendGoalEvent:
          // Remove the postfix "_SendGoal_Event"
          action_type =
            topic_type.substr(0, topic_type.length() - std::strlen("_SendGoal_Event"));
          break;
        case ActionInterfaceType::GetResultEvent:
          // Remove the postfix "_GetResult_Event"
          action_type =
            topic_type.substr(0, topic_type.length() - std::strlen("_GetResult_Event"));
          break;
        case ActionInterfaceType::Feedback:
          // Remove the postfix "_FeedbackMessage"
          action_type =
            topic_type.substr(0, topic_type.length() - std::strlen("_FeedbackMessage"));
          break;
        case ActionInterfaceType::CancelGoalEvent:
        case ActionInterfaceType::Status:
          break;
        default:
          throw std::runtime_error("Can't get action type. Unknown action interface type");
          break;
      }
      return action_type;
    }
  }

  return action_type;
}

ActionInterfaceType get_action_interface_type(const std::string & topic_name)
{
  for (auto &[topic_type_enum, postfix] : action_topic_to_postfix_map) {
    if (topic_name.length() > postfix.length() &&
      topic_name.compare(topic_name.length() - postfix.length(), postfix.length(), postfix) == 0)
    {
      return topic_type_enum;
    }
  }

  return ActionInterfaceType::Unknown;
}

std::vector<std::string> action_name_to_action_interface_names(const std::string & action_name)
{
  std::vector<std::string> action_topics;

  if (action_name.empty()) {
    return action_topics;
  }

  for (auto &[topic_type_enum, postfix] : action_topic_to_postfix_map) {
    action_topics.push_back(action_name + postfix);
  }

  return action_topics;
}
}  // namespace rosbag2_cpp
