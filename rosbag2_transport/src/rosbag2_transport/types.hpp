// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_TRANSPORT__TYPES_HPP_
#define ROSBAG2_TRANSPORT__TYPES_HPP_

#include <unordered_map>
#include <string>

namespace rosbag2_transport
{

typedef std::string TopicType;
typedef std::vector<std::string> TopicQoSProfiles;
struct TopicDetails {
  TopicType type;
  TopicQoSProfiles offered_qos_profiles;
};

typedef std::unordered_map<std::string, std::string> TopicsMap;
// typedef std::unordered_map<std::string, TopicDetails> TopicsMap;

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__TYPES_HPP_
