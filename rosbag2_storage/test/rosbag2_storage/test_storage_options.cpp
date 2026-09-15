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
  original.max_bag_files = 7;
  original.min_free_space_bytes = 1024ULL * 1024ULL * 1024ULL;
  original.min_free_space_percent = 12.5;
  original.low_free_space_action = rosbag2_storage::LowFreeSpaceAction::DELETE_OLDEST_FILES;
  original.max_cache_size = 1024;
  original.max_cache_duration = 30;
  original.storage_preset_profile = "profile";
  original.storage_config_uri = "config_uri";
  original.snapshot_mode = true;
  original.start_time_ns = 12345000;
  original.end_time_ns = 23456000;
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
  ASSERT_EQ(original.max_bag_files, reconstructed.max_bag_files);
  ASSERT_EQ(original.min_free_space_bytes, reconstructed.min_free_space_bytes);
  ASSERT_DOUBLE_EQ(original.min_free_space_percent, reconstructed.min_free_space_percent);
  ASSERT_EQ(original.low_free_space_action, reconstructed.low_free_space_action);
  ASSERT_EQ(original.max_cache_size, reconstructed.max_cache_size);
  ASSERT_EQ(original.max_cache_duration, reconstructed.max_cache_duration);
  ASSERT_EQ(original.storage_preset_profile, reconstructed.storage_preset_profile);
  ASSERT_EQ(original.storage_config_uri, reconstructed.storage_config_uri);
  ASSERT_EQ(original.snapshot_mode, reconstructed.snapshot_mode);
  ASSERT_EQ(original.start_time_ns, reconstructed.start_time_ns);
  ASSERT_EQ(original.end_time_ns, reconstructed.end_time_ns);
  ASSERT_EQ(original.custom_data, reconstructed.custom_data);
}

TEST(storage_options, test_invalid_numeric_value)
{
  YAML::Node node;
  node["uri"] = "some_uri";
  node["storage_id"] = "some_identification";
  node["max_bagfile_size"] = "non_numeric_value";
  try {
    node.as<rosbag2_storage::StorageOptions>();
    FAIL() << "Expected YAML::Exception to be thrown";
  } catch (const YAML::Exception & ex) {
    std::string error_msg = ex.what();
    EXPECT_THAT(error_msg, HasSubstr("max_bagfile_size"));
    EXPECT_THAT(error_msg, HasSubstr("Failed to convert field"));
  }
}

TEST(storage_options, low_free_space_action_string_conversion)
{
  using rosbag2_storage::LowFreeSpaceAction;
  EXPECT_EQ(rosbag2_storage::to_string(LowFreeSpaceAction::STOP), "stop");
  EXPECT_EQ(
    rosbag2_storage::to_string(LowFreeSpaceAction::DELETE_OLDEST_FILES), "delete_oldest_files");
  EXPECT_EQ(rosbag2_storage::low_free_space_action_from_string("stop"), LowFreeSpaceAction::STOP);
  EXPECT_EQ(
    rosbag2_storage::low_free_space_action_from_string("DELETE_OLDEST_FILES"),
    LowFreeSpaceAction::DELETE_OLDEST_FILES);
  EXPECT_THROW(
    rosbag2_storage::low_free_space_action_from_string("unknown"), std::invalid_argument);
}

TEST(storage_options, low_free_space_action_defaults_and_invalid_yaml_value)
{
  YAML::Node node;
  node["uri"] = "some_uri";
  auto storage_options = node.as<rosbag2_storage::StorageOptions>();
  EXPECT_EQ(storage_options.min_free_space_bytes, 0u);
  EXPECT_DOUBLE_EQ(storage_options.min_free_space_percent, 0.0);
  EXPECT_EQ(storage_options.low_free_space_action, rosbag2_storage::LowFreeSpaceAction::STOP);

  node["low_free_space_action"] = "not_an_action";
  try {
    node.as<rosbag2_storage::StorageOptions>();
    FAIL() << "Expected YAML::Exception to be thrown";
  } catch (const YAML::Exception & ex) {
    std::string error_msg = ex.what();
    EXPECT_THAT(error_msg, HasSubstr("low_free_space_action"));
    EXPECT_THAT(error_msg, HasSubstr("not_an_action"));
  }
}
