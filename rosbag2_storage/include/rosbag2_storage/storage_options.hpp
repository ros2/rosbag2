// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_STORAGE__STORAGE_OPTIONS_HPP_
#define ROSBAG2_STORAGE__STORAGE_OPTIONS_HPP_

#include <string>

namespace rosbag2_storage
{

struct StorageOptions
{
public:
  std::string uri;
  std::string storage_id;

  // The maximum size a bagfile can be, in bytes, before it is split.
  // A value of 0 indicates that bagfile splitting will not be used.
  uint64_t max_bagfile_size = 0;

  // The maximum duration a bagfile can be, in seconds, before it is split.
  // A value of 0 indicates that bagfile splitting will not be used.
  uint64_t max_bagfile_duration = 0;

  // The cache size indiciates how many messages can maximally be hold in cache
  // before these being written to disk.
  // Defaults to 0, and effectively disables the caching.
  uint64_t max_cache_size = 0;

  // Storage specific configuration file.
  // Defaults to empty string.
  std::string storage_config_uri = "";
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__STORAGE_OPTIONS_HPP_
