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

namespace rosbag2_cpp
{
bool is_service_event_topic(const std::string & topic, const std::string & topic_type);

std::string service_event_topic_name_to_service_name(const std::string & topic_name);

std::string service_event_topic_type_to_service_type(const std::string & topic_type);

size_t get_serialization_size_for_service_metadata_event();
}

#endif  // ROSBAG2_CPP__SERVICE_UTILS_HPP_
