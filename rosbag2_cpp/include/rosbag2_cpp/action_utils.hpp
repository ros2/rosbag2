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

#ifndef ROSBAG2_CPP__ACTION_UTILS_HPP_
#define ROSBAG2_CPP__ACTION_UTILS_HPP_

#include <string>
#include <vector>

#include "rosbag2_cpp/visibility_control.hpp"

namespace rosbag2_cpp
{
enum class TopicsInAction
{
  SendGoalEvent,
  CancelGoalEvent,
  GetResultEvent,
  Feedback,
  Status,
  Unknown
};

ROSBAG2_CPP_PUBLIC
bool
is_topic_related_to_action(const std::string & topic_name, const std::string & topic_type);

// Call this function after is_topic_related_to_action() return true
ROSBAG2_CPP_PUBLIC
std::string
action_topic_name_to_action_name(const std::string & topic_name);

// Call this function after is_topic_related_to_action() return true
// Note that cancel_goal event topic and status topic return ""
ROSBAG2_CPP_PUBLIC
std::string
action_topic_type_to_action_type(const std::string & topic_type);

ROSBAG2_CPP_PUBLIC
TopicsInAction
get_action_topic_type_from_topic_name(const std::string & topic_name);

ROSBAG2_CPP_PUBLIC
std::vector<std::string>
action_name_to_action_topic_name(const std::string & action_name);
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__ACTION_UTILS_HPP_
