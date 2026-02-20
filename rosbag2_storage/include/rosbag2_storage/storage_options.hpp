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

#include <cstdint>
#include <string>

#include "rosbag2_storage/visibility_control.hpp"
#include "rosbag2_storage/yaml.hpp"

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

  // The cache size indicates how many bytes of messages to hold in each buffer
  // of cache before writing to disk. Uses double buffering, so peak memory
  // usage can be up to twice this value.
  // A value of 0 disables caching and every write happens directly to disk.
  // Defaults to 100 MiB, matching the ros2 bag record CLI default.
  uint64_t max_cache_size = 100u * 1024u * 1024u;

  // Preset storage configuration. Preset settings can be overriden with
  // corresponding settings specified through storage_config_uri file
  std::string storage_preset_profile = "";

  // Storage specific configuration file.
  // Defaults to empty string.
  std::string storage_config_uri = "";

  // Enable snapshot mode.
  // Defaults to disabled.
  bool snapshot_mode = false;
};

}  // namespace rosbag2_storage

namespace YAML
{
template<>
struct ROSBAG2_STORAGE_PUBLIC convert<rosbag2_storage::StorageOptions>
{
  static Node encode(const rosbag2_storage::StorageOptions & storage_options);
  static bool decode(const Node & node, rosbag2_storage::StorageOptions & storage_options);
};
}  // namespace YAML

#endif  // ROSBAG2_STORAGE__STORAGE_OPTIONS_HPP_
