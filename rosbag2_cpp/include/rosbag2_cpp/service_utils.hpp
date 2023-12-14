// Copyright 2023 Sony Group Corporation.
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

#ifndef ROSBAG2_CPP__SERVICE_UTILS_HPP_
#define ROSBAG2_CPP__SERVICE_UTILS_HPP_

#include <string>

#include "rosbag2_cpp/visibility_control.hpp"

namespace rosbag2_cpp
{
ROSBAG2_CPP_PUBLIC
bool
is_service_event_topic(const std::string & topic, const std::string & topic_type);

// Call this function after is_service_event_topic() return true
ROSBAG2_CPP_PUBLIC
std::string
service_event_topic_name_to_service_name(const std::string & topic_name);

// Call this function after is_service_event_topic() return true
ROSBAG2_CPP_PUBLIC
std::string
service_event_topic_type_to_service_type(const std::string & topic_type);

ROSBAG2_CPP_PUBLIC
size_t
get_serialization_size_for_service_metadata_event();

ROSBAG2_CPP_PUBLIC
std::string
service_name_to_service_event_topic_name(const std::string & service_name);
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__SERVICE_UTILS_HPP_
