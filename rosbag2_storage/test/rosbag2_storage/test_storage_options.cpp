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

#include <gmock/gmock.h>

#include "rosbag2_storage/storage_options.hpp"

using namespace ::testing;  // NOLINT

TEST(storage_options, test_yaml_serialization)
{
  rosbag2_storage::StorageOptions original;
  original.uri = "some_uri";
  original.storage_id = "storage";
  original.max_bagfile_size = 5;
  original.max_bagfile_duration = 120;
  original.max_cache_size = 1024;
  original.storage_preset_profile = "profile";
  original.storage_config_uri = "config_uri";
  original.snapshot_mode = true;
  original.custom_data["key1"] = "value1";
  original.custom_data["key2"] = "value2";

  auto node = YAML::convert<rosbag2_storage::StorageOptions>().encode(original);

  std::stringstream serializer;
  serializer << node;

  auto reconstructed_node = YAML::Load(serializer.str());
  auto reconstructed = reconstructed_node.as<rosbag2_storage::StorageOptions>();

  ASSERT_EQ(original.uri, reconstructed.uri);
  ASSERT_EQ(original.storage_id, reconstructed.storage_id);
  ASSERT_EQ(original.max_bagfile_size, reconstructed.max_bagfile_size);
  ASSERT_EQ(original.max_bagfile_duration, reconstructed.max_bagfile_duration);
  ASSERT_EQ(original.max_cache_size, reconstructed.max_cache_size);
  ASSERT_EQ(original.storage_preset_profile, reconstructed.storage_preset_profile);
  ASSERT_EQ(original.storage_config_uri, reconstructed.storage_config_uri);
  ASSERT_EQ(original.snapshot_mode, reconstructed.snapshot_mode);
  ASSERT_EQ(original.custom_data, reconstructed.custom_data);
}
