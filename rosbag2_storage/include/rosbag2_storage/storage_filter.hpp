// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_STORAGE__STORAGE_FILTER_HPP_
#define ROSBAG2_STORAGE__STORAGE_FILTER_HPP_

#include <string>
#include <vector>

namespace rosbag2_storage
{

struct StorageFilter
{
  // Topic names to whitelist when reading a bag. Only messages matching these
  // specified topics will be returned. If list is empty, the filter is ignored
  // and all messages of topics are returned.
  std::vector<std::string> topics;

  // Service event topic names to whitelist when reading a bag. Only messages
  // matching these specified service event topics will be returned. If list
  // is empty, the filter is ignored and all messages of service event topics
  // are returned.
  std::vector<std::string> services_events;

  // Regular expression of topic names and service name to whitelist when
  // playing a bag.Only messages matching these specified topics or services
  // will be returned. If the string is empty, the filter is ignored and all
  // messages are returned.
  std::string regex = "";

  // Topic names to blacklist when reading a bag. Only messages unmatching these
  // topics will be returned. if list is empty, the filter is ignored and all
  // messages of topics are returned.
  std::vector<std::string> exclude_topics = {};

  // Service event topic names to blacklist when reading a bag. Only
  // messages unmatching these service event topics will be returned. If list
  // is empty, the filter is ignored and all messages of service event topics
  // are returned.
  std::vector<std::string> exclude_service_events = {};

  // Regular expression of topic names and service events names to blacklist when
  // playing a bag. Only messages not matching these topics and service events will
  // be returned. If the string is empty, the filter is ignored and all messages
  // of topics and service events are returned.
  std::string regex_to_exclude = "";
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__STORAGE_FILTER_HPP_
