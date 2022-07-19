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
  // and all messages are returned.
  std::vector<std::string> topics;

  // Regular expression of topic names to whitelist when playing a bag.
  // Only messages matching these specified topics will be played.
  // If list is empty, the filter is ignored and all messages are played.
  std::string topics_regex = "";

  // Regular expression of topic names to exclude when playing a bag.
  // Only messages not matching these specified topics will be played.
  // If list is empty, the filter is ignored and all messages are played.
  std::string topics_regex_to_exclude = "";
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__STORAGE_FILTER_HPP_
