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
/// \brief Enum class for action introspection interface types.
enum class ActionInterfaceType
{
  SendGoalEvent,
  CancelGoalEvent,
  GetResultEvent,
  Feedback,
  Status,
  Unknown
};

/// \brief Check if the topic belong to the action introspection interface.
/// \param topic_name Topic name.
/// \param topic_type Topic type.
/// \return Boolean value indicating if the topic belongs to the action introspection interface.
ROSBAG2_CPP_PUBLIC
bool
is_topic_belong_to_action(const std::string & topic_name, const std::string & topic_type);

/// \brief Check if the topic belong to the action introspection interface.
/// \param action_interface_type Action interface type given from the
/// get_action_interface_type(const std::string & topic_name) function call.
/// \param topic_type
/// \return Boolean value indicating if the topic belongs to the action introspection interface.
ROSBAG2_CPP_PUBLIC
bool
is_topic_belong_to_action(
  ActionInterfaceType action_interface_type, const std::string & topic_type);

/// \brief Transform the action's introspection topic name to the short action name.
/// \param topic_name Topic name.
/// \note Call this function after is_topic_belong_to_action() returns true
/// \return String with short action name.
/// \example if topic_name is "/fibonacci/_action/send_goal/_service_event" the function will
/// return the short action name "/fibonacci"
ROSBAG2_CPP_PUBLIC
std::string
action_interface_name_to_action_name(const std::string & topic_name);

/// \brief Extract the original action type from the action's introspection type.
/// \note Call this function after is_topic_belong_to_action() returns true
/// \param topic_type Topic type name.
/// \return Original action type name aka "example_interfaces/action/Fibonacci" if action interface
//// type is SendGoalEvent, GetResultEvent or Feedback message, if action interface type is
/// CancelGoalEvent or Status, return empty string.
/// \throw std::runtime_error for unknown action interface type
ROSBAG2_CPP_PUBLIC
std::string
get_action_type_for_info(const std::string & topic_type);

/// \brief Extract the action interface type from the action's introspection topic name.
/// \param topic_name Topic name.
/// \return ActionInterfaceType enum value.
ROSBAG2_CPP_PUBLIC
ActionInterfaceType
get_action_interface_type(const std::string & topic_name);

/// \brief Transform the action name returned from the `action_interface_name_to_action_name(..)
/// to the list of the action introspection topics.
/// \param action_name Action name returned from the `action_interface_name_to_action_name(..)`.
/// \return Vector of action introspection topics corresponding to the action name.
ROSBAG2_CPP_PUBLIC
std::vector<std::string>
action_name_to_action_interface_names(const std::string & action_name);
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__ACTION_UTILS_HPP_
