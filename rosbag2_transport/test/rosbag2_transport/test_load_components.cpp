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

#include <memory>
#include <string>

#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "composition_interfaces/srv/list_nodes.hpp"

#include "rclcpp_components/component_manager.hpp"
#include "rclcpp_components/component_manager_isolated.hpp"

using namespace std::chrono_literals;

class TestComponentManager : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST_F(TestComponentManager, test_load_components)
{
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = rclcpp::Node::make_shared("test_component_manager");
  std::shared_ptr<rclcpp_components::ComponentManager> manager;
  using ComponentManagerIsolated =
    rclcpp_components::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>;
  manager = std::make_shared<ComponentManagerIsolated>(exec);

  exec->add_node(manager);
  exec->add_node(node);

  auto composition_client = node->create_client<composition_interfaces::srv::LoadNode>(
    "/ComponentManager/_container/load_node");

  if (!composition_client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  {
    std::string path{_SRC_RESOURCES_DIR_PATH "/params_player.yaml"};
    auto pm = rclcpp::parameter_map_from_yaml_file(path);
    auto pl = rclcpp::parameters_from_map(pm, "/player_params_node");
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rosbag2_transport";
    request->plugin_name = "rosbag2_transport::Player";
    for (auto p : pl) {
      request->parameters.push_back(p.to_parameter_msg());
    }

    rclcpp::Parameter qos_profile_overrides_path("qos_profile_overrides_path",
      rclcpp::ParameterValue(_SRC_RESOURCES_DIR_PATH "/qos_profile_overrides.yaml"));

    rclcpp::Parameter uri("uri",
      rclcpp::ParameterValue(_SRC_RESOURCES_DIR_PATH "//sqlite3/test_bag_for_seek"));

    request->parameters.push_back(qos_profile_overrides_path.to_parameter_msg());
    request->parameters.push_back(uri.to_parameter_msg());

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    auto result = future.get();
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result->success, true);
    EXPECT_EQ(result->error_message, "");
    EXPECT_EQ(result->full_node_name, "/rosbag2_player");
    EXPECT_EQ(result->unique_id, 1u);
  }

  {
    std::string path{_SRC_RESOURCES_DIR_PATH "/params_recorder.yaml"};
    auto pm = rclcpp::parameter_map_from_yaml_file(path);
    auto pl = rclcpp::parameters_from_map(pm, "/recorder_params_node");
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rosbag2_transport";
    request->plugin_name = "rosbag2_transport::Recorder";
    for (auto p : pl) {
      request->parameters.push_back(p.to_parameter_msg());
    }

    rclcpp::Parameter qos_profile_overrides_path("qos_profile_overrides_path",
      rclcpp::ParameterValue(_SRC_RESOURCES_DIR_PATH "/qos_profile_overrides.yaml"));

    request->parameters.push_back(qos_profile_overrides_path.to_parameter_msg());

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    auto result = future.get();
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result->success, true);
    EXPECT_EQ(result->error_message, "");
    EXPECT_EQ(result->full_node_name, "/rosbag2_recorder");
    EXPECT_EQ(result->unique_id, 2u);
  }
}
