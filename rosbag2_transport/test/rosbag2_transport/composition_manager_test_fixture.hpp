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

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include "composition_interfaces/srv/load_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "rclcpp_components/component_manager_isolated.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"

class CompositionManagerTestFixture
  : public rosbag2_test_common::ParametrizedTemporaryDirectoryFixture
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    auto bag_name = get_test_name() + "_" + GetParam();
    root_bag_path_ = std::filesystem::path(temporary_dir_path_) / bag_name;

    // Clean up potentially leftover bag files.
    // There may be leftovers if the system reallocates a temp directory
    // used by a previous test execution and the test did not have a clean exit.
    std::filesystem::remove_all(root_bag_path_);

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    node_ = rclcpp::Node::make_shared("test_component_manager");
    using ComponentManagerIsolated =
      rclcpp_components::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>;
    composition_manager_ = std::make_shared<ComponentManagerIsolated>(exec_);

    exec_->add_node(composition_manager_);
    exec_->add_node(node_);

    load_node_client_ = node_->create_client<composition_interfaces::srv::LoadNode>(
      "/ComponentManager/_container/load_node");

    if (!load_node_client_->wait_for_service(std::chrono::seconds(20))) {
      ASSERT_TRUE(false) << "service not available after waiting";
    }

    unload_node_client_ = node_->create_client<composition_interfaces::srv::UnloadNode>(
      "/ComponentManager/_container/unload_node");

    if (!unload_node_client_->wait_for_service(std::chrono::seconds(20))) {
      ASSERT_TRUE(false) << "service not available after waiting";
    }
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    composition_manager_.reset();  // Need to force destruction to invoke composed recorder
    // destructor before trying to delete files which is currently opened for writing.
    std::filesystem::remove_all(root_bag_path_);
  }

  void unload_node(uint64_t node_id)
  {
    auto unload_node_request = std::make_shared<composition_interfaces::srv::UnloadNode::Request>();
    unload_node_request->unique_id = node_id;
    auto unload_node_future = unload_node_client_->async_send_request(unload_node_request);
    // Wait for the response
    auto unload_node_ret =
      exec_->spin_until_future_complete(unload_node_future, std::chrono::seconds(10));
    auto unload_node_response = unload_node_future.get();
    EXPECT_EQ(unload_node_ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(unload_node_response->success, true);
    EXPECT_EQ(unload_node_response->error_message, "");
  }

  std::string get_test_name() const
  {
    const auto * test_info = UnitTest::GetInstance()->current_test_info();
    std::string test_name = test_info->name();
    // Replace any slashes in the test name, since it is used in paths
    std::replace(test_name.begin(), test_name.end(), '/', '_');
    return test_name;
  }

protected:
  std::filesystem::path root_bag_path_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<rclcpp::Client<composition_interfaces::srv::LoadNode>> load_node_client_;
  std::shared_ptr<rclcpp::Client<composition_interfaces::srv::UnloadNode>> unload_node_client_;
  std::shared_ptr<rclcpp_components::ComponentManager> composition_manager_;
};

#endif  // ROSBAG2_TRANSPORT__COMPOSITION_MANAGER_TEST_FIXTURE_HPP_
