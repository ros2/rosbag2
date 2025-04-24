// Copyright 2025 Sony Group Corporation.
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

#ifndef ROSBAG2_TEST_COMMON__ACTION_SERVER_MANAGER_HPP_
#define ROSBAG2_TEST_COMMON__ACTION_SERVER_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/executors.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace rosbag2_test_common
{
class ActionServerManager
{
public:
  ActionServerManager()
  : action_server_node_(std::make_shared<rclcpp::Node>(
        "action_server_manager_" + std::to_string(rclcpp::Clock().now().nanoseconds()),
        rclcpp::NodeOptions()
        .start_parameter_event_publisher(false)
        .enable_rosout(false)
        .start_parameter_services(false))),
    check_action_server_ready_node_(std::make_shared<rclcpp::Node>(
        "check_service_ready_node_" + std::to_string(rclcpp::Clock().now().nanoseconds()),
        rclcpp::NodeOptions()
        .start_parameter_event_publisher(false)
        .enable_rosout(false)
        .start_parameter_services(false)))
  {
  }

  ~ActionServerManager()
  {
    exec_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  template<typename ActionT>
  void setup_action(
    std::string action_name,
    size_t & request_count,
    size_t & cancel_count, std::chrono::milliseconds exec_goal_time = 200ms)
  {
    auto handle_goal = [&request_count](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const typename ActionT::Goal> goal)
      {
        (void) uuid;
        (void) goal;
        ++request_count;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

    auto handle_cancel = [&cancel_count](
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
      {
        (void)goal_handle;
        ++cancel_count;
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto handle_accepted = [this, exec_goal_time](
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
      {
        (void)goal_handle;
        auto execute_in_thread =
          [this, goal_handle, exec_goal_time]()
          {
            return this->action_execute(goal_handle, exec_goal_time);
          };
        std::thread{execute_in_thread}.detach();
      };

    auto server = rclcpp_action::create_server<ActionT>(
      action_server_node_, action_name, handle_goal, handle_cancel, handle_accepted);
    server->configure_introspection(
      action_server_node_->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
    action_servers_.emplace(action_name, server);

    auto client = rclcpp_action::create_client<ActionT>(
      check_action_server_ready_node_, action_name);
    client->configure_introspection(
      action_server_node_->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);

    action_clients_.emplace_back(client);
  }

  void run_action_servers()
  {
    exec_.add_node(action_server_node_);
    thread_ = std::thread(
      [this]() {
        exec_.spin();
      });
  }

  bool all_action_servers_ready()
  {
    for (auto client : action_clients_) {
      if (!client->wait_for_action_server(std::chrono::seconds(2))) {
        return false;
      }
    }
    return true;
  }

private:
  std::shared_ptr<rclcpp::Node> action_server_node_;
  std::shared_ptr<rclcpp::Node> check_action_server_ready_node_;
  std::unordered_map<std::string, typename rclcpp_action::ServerBase::SharedPtr> action_servers_;
  std::vector<typename rclcpp_action::ClientBase::SharedPtr> action_clients_;
  std::thread thread_;
  rclcpp::executors::SingleThreadedExecutor exec_;

  template<typename ActionT>
  void action_execute(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle,
    std::chrono::milliseconds exec_goal_time)
  {
    auto result = std::make_shared<typename ActionT::Result>();
    std::this_thread::sleep_for(exec_goal_time);
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }
    goal_handle->succeed(result);
  }
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__ACTION_SERVER_MANAGER_HPP_
