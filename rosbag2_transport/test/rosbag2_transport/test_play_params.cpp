// Copyright 2023, Patrick Roncagliolo.
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

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "mock_player.hpp"
#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/play_options.hpp"

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

  auto node = std::make_shared<MockPlayer>("player_params_node", opts);
  auto play_options = node->get_play_options();
  auto storage_options = node->get_storage_options();
  YAML::Node yaml_play_opt = YAML::convert<rosbag2_transport::PlayOptions>().encode(play_options);
  YAML::Node yaml_storage_opt = YAML::convert<rosbag2_storage::StorageOptions>().encode(
    storage_options);

  auto param_node = YAML::LoadFile(_SRC_RESOURCES_DIR_PATH "/player_node_params.yaml");
  auto qos_node = YAML::LoadFile(_SRC_RESOURCES_DIR_PATH "/qos_profile_overrides.yaml");

  EXPECT_EQ(play_options.read_ahead_queue_size, 3);
  EXPECT_EQ(play_options.node_prefix, "test");
  EXPECT_EQ(play_options.rate, 13.0);
  std::vector<std::string> topics_to_filter {"/foo", "/bar"};
  EXPECT_EQ(play_options.topics_to_filter, topics_to_filter);
  EXPECT_EQ(play_options.topics_regex_to_filter, "[xyz]/topic");
  EXPECT_EQ(play_options.topics_regex_to_exclude, "[abc]/topic");
  EXPECT_EQ(play_options.loop, false);
  EXPECT_EQ(play_options.clock_publish_frequency, 19.0);
  std::vector<std::string> clock_trigger_topics {"/triggers/clock"};
  EXPECT_EQ(play_options.clock_trigger_topics, clock_trigger_topics);
  EXPECT_EQ(play_options.delay.nanoseconds(), 1);
  EXPECT_EQ(play_options.playback_duration.seconds(), -1);
  // TODO(roncapat): compare the other params (from file vs from struct)
}