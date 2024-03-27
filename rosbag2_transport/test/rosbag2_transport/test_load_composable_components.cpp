// Copyright 2023, Patrick Roncagliolo
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

#include "composition_manager_test_fixture.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace fs = std::filesystem;

TEST_P(CompositionManagerTestFixture, test_load_play_component)
{
  std::string path{_SRC_RESOURCES_DIR_PATH "/player_node_params.yaml"};
  auto pm = rclcpp::parameter_map_from_yaml_file(path);
  auto pl = rclcpp::parameters_from_map(pm, "/player_params_node");
  auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
  request->package_name = "rosbag2_transport";
  request->plugin_name = "rosbag2_transport::Player";
  for (const auto & p : pl) {
    request->parameters.push_back(p.to_parameter_msg());
  }

  rclcpp::Parameter qos_profile_overrides_path("play.qos_profile_overrides_path",
    rclcpp::ParameterValue(_SRC_RESOURCES_DIR_PATH "/qos_profile_overrides.yaml"));

  const std::string uri_str = (fs::path(
      _SRC_RESOURCES_DIR_PATH) / GetParam() / "test_bag_for_seek").generic_string();
  rclcpp::Parameter uri("storage.uri", rclcpp::ParameterValue(uri_str));
  rclcpp::Parameter storage_id("storage.storage_id", GetParam());

  request->parameters.push_back(qos_profile_overrides_path.to_parameter_msg());
  request->parameters.push_back(uri.to_parameter_msg());
  request->parameters.push_back(storage_id.to_parameter_msg());

  auto future = load_node_client_->async_send_request(request);
  auto ret = exec_->spin_until_future_complete(future, 10s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  auto result = future.get();
  EXPECT_EQ(result->success, true);
  EXPECT_EQ(result->error_message, "");
  EXPECT_EQ(result->full_node_name, "/rosbag2_player");
  EXPECT_EQ(result->unique_id, 1u);

  // Unload composed player node
  unload_node(result->unique_id);
}

TEST_P(CompositionManagerTestFixture, test_load_record_component)
{
  std::string path{_SRC_RESOURCES_DIR_PATH "/recorder_node_params.yaml"};
  auto pm = rclcpp::parameter_map_from_yaml_file(path);
  auto pl = rclcpp::parameters_from_map(pm, "/recorder_params_node");
  auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
  request->package_name = "rosbag2_transport";
  request->plugin_name = "rosbag2_transport::Recorder";
  for (const auto & p : pl) {
    request->parameters.push_back(p.to_parameter_msg());
  }

  rclcpp::Parameter qos_profile_overrides_path("record.qos_profile_overrides_path",
    rclcpp::ParameterValue(_SRC_RESOURCES_DIR_PATH "/qos_profile_overrides.yaml"));
  rclcpp::Parameter uri("storage.uri", rclcpp::ParameterValue(root_bag_path_.generic_string()));
  rclcpp::Parameter storage_id("storage.storage_id", GetParam());

  request->parameters.push_back(qos_profile_overrides_path.to_parameter_msg());
  request->parameters.push_back(uri.to_parameter_msg());
  request->parameters.push_back(storage_id.to_parameter_msg());

  auto future = load_node_client_->async_send_request(request);
  auto ret = exec_->spin_until_future_complete(future, 10s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  auto result = future.get();
  EXPECT_EQ(result->success, true);
  EXPECT_EQ(result->error_message, "");
  EXPECT_EQ(result->full_node_name, "/rosbag2_recorder");
  EXPECT_EQ(result->unique_id, 1u);

  // Unload composed recorder node
  unload_node(result->unique_id);
}

INSTANTIATE_TEST_SUITE_P(
  ParametrizedLoadComposableComponentsTests,
  CompositionManagerTestFixture,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
