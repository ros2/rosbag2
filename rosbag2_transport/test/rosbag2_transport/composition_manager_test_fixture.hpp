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

#ifndef ROSBAG2_TRANSPORT__COMPOSITION_MANAGER_TEST_FIXTURE_HPP_
#define ROSBAG2_TRANSPORT__COMPOSITION_MANAGER_TEST_FIXTURE_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "composition_interfaces/srv/load_node.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "rclcpp_components/component_manager_isolated.hpp"

class CompositionManagerTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    node_ = rclcpp::Node::make_shared("test_component_manager");
    using ComponentManagerIsolated =
      rclcpp_components::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>;
    composition_manager_ = std::make_shared<ComponentManagerIsolated>(exec_);

    exec_->add_node(composition_manager_);
    exec_->add_node(node_);

    composition_client_ = node_->create_client<composition_interfaces::srv::LoadNode>(
      "/ComponentManager/_container/load_node");

    if (!composition_client_->wait_for_service(std::chrono::seconds(20))) {
      ASSERT_TRUE(false) << "service not available after waiting";
    }
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<rclcpp::Client<composition_interfaces::srv::LoadNode>> composition_client_;
  std::shared_ptr<rclcpp_components::ComponentManager> composition_manager_;
};

#endif  // ROSBAG2_TRANSPORT__COMPOSITION_MANAGER_TEST_FIXTURE_HPP_
