// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <string>

#include "rosbag2_storage/storage_options.hpp"

namespace YAML
{

Node convert<rosbag2_storage::StorageOptions>::encode(
  const rosbag2_storage::StorageOptions & storage_options)
{
  Node node;
  node["uri"] = storage_options.uri;
  node["storage_id"] = storage_options.storage_id;
  node["max_bagfile_size"] = storage_options.max_bagfile_size;
  node["max_bagfile_duration"] = storage_options.max_bagfile_duration;
  node["max_cache_size"] = storage_options.max_cache_size;
  node["storage_preset_profile"] = storage_options.storage_preset_profile;
  node["storage_config_uri"] = storage_options.storage_config_uri;
  node["snapshot_mode"] = storage_options.snapshot_mode;
  return node;
}

bool convert<rosbag2_storage::StorageOptions>::decode(
  const Node & node, rosbag2_storage::StorageOptions & storage_options)
{
  storage_options.uri = node["uri"].as<std::string>();
  optional_assign<std::string>(node, "storage_id", storage_options.storage_id);
  optional_assign<uint64_t>(node, "max_bagfile_size", storage_options.max_bagfile_size);
  optional_assign<uint64_t>(node, "max_bagfile_duration", storage_options.max_bagfile_duration);
  optional_assign<uint64_t>(node, "max_cache_size", storage_options.max_cache_size);
  optional_assign<std::string>(
    node, "storage_preset_profile", storage_options.storage_preset_profile);
  optional_assign<std::string>(node, "storage_config_uri", storage_options.storage_config_uri);
  optional_assign<bool>(node, "snapshot_mode", storage_options.snapshot_mode);
  return true;
}

}  // namespace YAML
