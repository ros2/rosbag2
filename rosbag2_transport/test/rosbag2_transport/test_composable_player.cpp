// Copyright 2023, Patrick Roncagliolo and Michael Orlov
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

#include <memory>
#include <string>
#include <vector>

#include "mock_player.hpp"
#include "rosbag2_play_test_fixture.hpp"

TEST_F(RosBag2PlayTestFixture, parse_parameter_from_file) {
  // _SRC_RESOURCES_DIR_PATH defined in CMakeLists.txt
  rclcpp::NodeOptions opts;
  opts.arguments(
  {
    "--ros-args",
    "--params-file", _SRC_RESOURCES_DIR_PATH "/player_node_params.yaml"
  });
  opts.append_parameter_override(
    "qos_profile_overrides_path",
    _SRC_RESOURCES_DIR_PATH "/qos_profile_overrides.yaml");
  opts.append_parameter_override(
    "uri",
    _SRC_RESOURCES_DIR_PATH "/sqlite3/test_bag_for_seek");

  auto player = std::make_shared<MockPlayer>("player_params_node", opts);
  auto play_options = player->get_play_options();
  auto storage_options = player->get_storage_options();

  EXPECT_EQ(play_options.read_ahead_queue_size, 3);
  EXPECT_EQ(play_options.node_prefix, "test");
  EXPECT_EQ(play_options.rate, 13.0);
  std::vector<std::string> topics_to_filter {"/foo", "/bar"};
  EXPECT_EQ(play_options.topics_to_filter, topics_to_filter);
  EXPECT_EQ(play_options.topics_regex_to_filter, "[xyz]/topic");
  EXPECT_EQ(play_options.topics_regex_to_exclude, "[abc]/topic");
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides{
    std::pair{
      "/overrided_topic_qos",
      rclcpp::QoS{rclcpp::KeepLast(10)}.reliable().durability_volatile()}
  };
  EXPECT_EQ(play_options.topic_qos_profile_overrides, topic_qos_profile_overrides);
  EXPECT_EQ(play_options.loop, false);
  EXPECT_EQ(play_options.clock_publish_frequency, 19.0);
  std::vector<std::string> clock_trigger_topics {"/triggers/clock"};
  EXPECT_EQ(play_options.clock_trigger_topics, clock_trigger_topics);
  EXPECT_EQ(play_options.delay.nanoseconds(), 1);
  EXPECT_FLOAT_EQ(play_options.playback_duration.seconds(), -1);
  EXPECT_EQ(play_options.playback_until_timestamp, -2500000000);
  EXPECT_EQ(play_options.start_offset, 999999999);
  EXPECT_EQ(play_options.wait_acked_timeout, -999999999);
  EXPECT_EQ(play_options.disable_loan_message, false);

  EXPECT_EQ(
    storage_options.uri,
    _SRC_RESOURCES_DIR_PATH "/sqlite3/test_bag_for_seek");
  EXPECT_EQ(storage_options.storage_id, "sqlite3");
  EXPECT_EQ(storage_options.storage_config_uri, "");
  EXPECT_EQ(storage_options.max_bagfile_size, 12345);
  EXPECT_EQ(storage_options.max_bagfile_duration, 54321);
  EXPECT_EQ(storage_options.max_cache_size, 9898);
  EXPECT_EQ(storage_options.storage_preset_profile, "resilient");
  EXPECT_EQ(storage_options.snapshot_mode, false);
  std::unordered_map<std::string, std::string> custom_data{
    std::pair{"key1", "value1"},
    std::pair{"key2", "value2"}
  };
  EXPECT_EQ(storage_options.custom_data, custom_data);
}
