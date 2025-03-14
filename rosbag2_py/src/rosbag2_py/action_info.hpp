// Copyright 2025 Open Source Robotics Foundation, Inc.
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


#ifndef ROSBAG2_PY__ACTION_INFO_HPP_
#define ROSBAG2_PY__ACTION_INFO_HPP_

#include <string>

namespace rosbag2_py
{

struct ActionMetadata
{
  std::string name;
  std::string type;
  std::string serialization_format;
};

struct ActionInformation
{
  ActionMetadata action_metadata;
  size_t send_goal_event_message_count = 0;
  size_t cancel_goal_event_message_count = 0;
  size_t get_result_event_message_count = 0;
  size_t feedback_message_count = 0;
  size_t status_message_count = 0;
};

}  // namespace rosbag2_py

#endif  // ROSBAG2_PY__ACTION_INFO_HPP_
