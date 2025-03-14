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
const std::unordered_map<TopicsInAction, std::string> ActionTopicPostfix = {
  {TopicsInAction::SendGoalEvent, "/_action/send_goal/_service_event"},
  {TopicsInAction::CancelGoalEvent, "/_action/cancel_goal/_service_event"},
  {TopicsInAction::GetResultEvent, "/_action/get_result/_service_event"},
  {TopicsInAction::Feedback, "/_action/feedback"},
  {TopicsInAction::Status, "/_action/status"}
};

// The regex pattern of the action internal topics and service event topics
const std::unordered_map<TopicsInAction, std::string> ActionTopicTypeRegex = {
  {TopicsInAction::SendGoalEvent, ".+/action/.+SendGoal_Event$"},
  {TopicsInAction::CancelGoalEvent, "^action_msgs/srv/CancelGoal_Event$"},
  {TopicsInAction::GetResultEvent, ".+/action/.+GetResult_Event$"},
  {TopicsInAction::Feedback, ".+/action/.+_FeedbackMessage$"},
  {TopicsInAction::Status, "^action_msgs/msg/GoalStatusArray$"}
};

const size_t kMinActionTopicPostfixLen = ActionTopicPostfix.at(TopicsInAction::Status).length();

bool is_topic_related_to_action(const std::string & topic_name, const std::string & topic_type)
{
  TopicsInAction topic = TopicsInAction::Unknown;
  if (topic_name.length() <= kMinActionTopicPostfixLen) {
    return false;
  } else {
    for (auto &[topic_type_enum, postfix] : ActionTopicPostfix) {
      if (topic_name.length() > postfix.length() &&
        topic_name.compare(
          topic_name.length() - postfix.length(), postfix.length(), postfix) == 0)
      {
        topic = topic_type_enum;
        break;
      }
    }
  }

  if (topic == TopicsInAction::Unknown) {
    return false;
  }

  std::regex pattern(ActionTopicTypeRegex.at(topic));
  return std::regex_search(topic_type, pattern);
}

std::string action_topic_name_to_action_name(const std::string & topic_name)
{
  std::string action_name;
  if (topic_name.length() <= kMinActionTopicPostfixLen) {
    return action_name;
  } else {
    for (auto &[topic_type_enum, postfix] : ActionTopicPostfix) {
      if (topic_name.length() > postfix.length() &&
        topic_name.compare(
          topic_name.length() - postfix.length(), postfix.length(), postfix) == 0)
      {
        action_name = topic_name.substr(0, topic_name.length() - postfix.length());
        break;
      }
    }
  }

  return action_name;
}

std::string action_topic_type_to_action_type(const std::string & topic_type)
{
  std::string service_type;

  for (auto &[topic_type_enum, regex] : ActionTopicTypeRegex) {
    std::regex pattern(regex);
    if (std::regex_search(topic_type, pattern)) {
      switch (topic_type_enum) {
        case TopicsInAction::SendGoalEvent:
          // Remove the postfix "_SendGoal_Event"
          service_type =
            topic_type.substr(0, topic_type.length() - std::strlen("_SendGoal_Event"));
          break;
        case TopicsInAction::GetResultEvent:
          // Remove the postfix "_GetResult_Event"
          service_type =
            topic_type.substr(0, topic_type.length() - std::strlen("_GetResult_Event"));
          break;
        case TopicsInAction::Feedback:
          // Remove the postfix "_FeedbackMessage"
          service_type =
            topic_type.substr(0, topic_type.length() - std::strlen("_FeedbackMessage"));
          break;
        case TopicsInAction::CancelGoalEvent:
        case TopicsInAction::Status:
        default:
          break;
      }
      return service_type;
    }
  }

  return service_type;
}

TopicsInAction get_action_topic_type_from_topic_name(const std::string & topic_name)
{
  for (auto &[topic_type_enum, postfix] : ActionTopicPostfix) {
    if (topic_name.length() > postfix.length() &&
      topic_name.compare(
        topic_name.length() - postfix.length(), postfix.length(), postfix) == 0)
    {
      return topic_type_enum;
    }
  }

  return TopicsInAction::Unknown;
}

std::vector<std::string> action_name_to_action_topic_name(const std::string & action_name)
{
  std::vector<std::string> action_topics;

  if (action_name.empty()) {
    return action_topics;
  }

  for (auto &[topic_type_enum, postfix] : ActionTopicPostfix) {
    action_topics.push_back(action_name + postfix);
  }

  return action_topics;
}
}  // namespace rosbag2_cpp
