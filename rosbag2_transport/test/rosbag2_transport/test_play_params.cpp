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
    "--params-file", _SRC_RESOURCES_DIR_PATH "/params.yaml"
  });

  auto node = std::make_shared<MockPlayer>("player_params_node", opts);
  auto play_options = node->retrieve_play_options();
  auto storage_options = node->retrieve_storage_options();
  YAML::Node yaml_play_opt = YAML::convert<rosbag2_transport::PlayOptions>().encode(play_options);
  YAML::Node yaml_storage_opt = YAML::convert<rosbag2_storage::StorageOptions>().encode(
    storage_options);

  YAML::Emitter emitter;
  emitter << yaml_play_opt;
  std::cout << "Node :" << emitter.c_str() << std::endl;

  // TODO(roncapat): compare YAML trees (from file vs from struct)
}
