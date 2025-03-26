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

#ifndef ROSBAG2_TEST_COMMON__ACTION_CLIENT_MANAGER_HPP_
#define ROSBAG2_TEST_COMMON__ACTION_CLIENT_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl/service_introspection.h"

#include "rclcpp/rclcpp.hpp"  // rclcpp must be included before the Windows specific includes.
#include "rclcpp_action/rclcpp_action.hpp"

#include "test_msgs/action/fibonacci.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace rosbag2_test_common
{
template<typename ServiceT>
class ActionClientManager : public rclcpp::Node
{
public:
  using Fibonacci = test_msgs::action::Fibonacci;
  using ServerGoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
  using ClientGoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit ActionClientManager(
    std::string action_name,
    std::chrono::seconds exec_goal_time = 1s,
    size_t number_of_clients = 1,
    bool enable_action_server_introspection = true,
    bool enable_action_client_introspection = false)
  : Node("action_client_manager_" + std::to_string(rclcpp::Clock().now().nanoseconds()),
      rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(
        false).enable_rosout(false)),
    action_name_(std::move(action_name)),
    number_of_clients_(number_of_clients),
    enable_action_server_introspection_(enable_action_server_introspection),
    enable_action_client_introspection_(enable_action_client_introspection)
  {
    create_action_server(exec_goal_time);

    create_action_client(number_of_clients);

    rcl_service_introspection_state_t introspection_state;
    if (enable_action_server_introspection_) {
      introspection_state = RCL_SERVICE_INTROSPECTION_CONTENTS;
    } else {
      introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
    }
    action_server_->configure_introspection(
      get_clock(), rclcpp::SystemDefaultsQoS(), introspection_state);

    if (enable_action_client_introspection_) {
      introspection_state = RCL_SERVICE_INTROSPECTION_CONTENTS;
    } else {
      introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
    }

    for (auto & action_client : action_clients_) {
      action_client->configure_introspection(
        get_clock(), rclcpp::SystemDefaultsQoS(), introspection_state);
    }
  }

  void create_action_server(std::chrono::seconds exec_goal_time)
  {
    auto handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Fibonacci::Goal> goal)
      {
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

    auto handle_cancel = [this](
      const std::shared_ptr<ServerGoalHandleFibonacci> goal_handle)
      {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto handle_accepted = [this, exec_goal_time](
      const std::shared_ptr<ServerGoalHandleFibonacci> goal_handle)
      {
        // this needs to return quickly to avoid blocking the executor,
        // so we declare a lambda function to be called inside a new thread
        auto execute_in_thread =
          [this, goal_handle, exec_goal_time]() {
            return this->execute(goal_handle, exec_goal_time);
          };
        std::thread{execute_in_thread}.detach();
      };

    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      action_name_,
      handle_goal,
      handle_cancel,
      handle_accepted);
  }

  void create_action_client(size_t number_of_clients)
  {
    for (size_t i = 0; i < number_of_clients_; i++) {
      auto action_client = rclcpp_action::create_client<Fibonacci>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        action_name_);

      action_clients_.emplace_back(action_client);
    }
  }

  bool check_action_server_ready()
  {
    for (auto & action_client : action_clients_) {
      if (!action_client->action_server_is_ready()) {
        return false;
      }
    }
    return true;
  }

  bool wait_for_action_server_to_be_ready(
    std::chrono::duration<double> timeout = std::chrono::seconds(5))
  {
    using clock = std::chrono::system_clock;
    auto start = clock::now();
    while (!check_action_server_ready() && (clock::now() - start) < timeout) {
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return check_action_server_ready();
  }

  bool send_goal(
    bool cancel_goal_after_accept = false,
    std::chrono::duration<double> timeout = std::chrono::seconds(10))
  {
    if (!check_action_server_ready()) {
      return false;
    }

    for (auto & action_client : action_clients_) {
      auto goal_msg = Fibonacci::Goal();
      goal_msg.order = 3;
      auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

      send_goal_options.goal_response_callback = [this, action_client, cancel_goal_after_accept](
        const ClientGoalHandleFibonacci::SharedPtr & goal_handle)
        {
          if (goal_handle) {
            if (cancel_goal_after_accept) {
              action_client->async_cancel_goal(goal_handle);
            }
          }
        };

      send_goal_options.feedback_callback = [this](
        ClientGoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
        {
          (void)feedback;
        };

      send_goal_options.result_callback = [this](
        const ClientGoalHandleFibonacci::WrappedResult & result)
        {
          (void)result;
        };

      auto send_goal_future = action_client->async_send_goal(goal_msg, send_goal_options);

      if (rclcpp::executors::spin_node_until_future_complete(
          exec_, get_node_base_interface(), send_goal_future, 1s) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        return false;
      }

      auto goal_handle = send_goal_future.get();
      if (!goal_handle) {
        return false;
      }

      auto result_future = action_client->async_get_result(goal_handle);
      if (rclcpp::executors::spin_node_until_future_complete(
          exec_, get_node_base_interface(), result_future, timeout) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        return false;
      }
    }
    return true;
  }

private:
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  std::vector<rclcpp_action::Client<Fibonacci>::SharedPtr> action_clients_;
  const std::string action_name_;
  size_t number_of_clients_;
  bool enable_action_server_introspection_;
  bool enable_action_client_introspection_;

  void execute(
    const std::shared_ptr<ServerGoalHandleFibonacci> goal_handle,
    std::chrono::seconds exec_goal_time)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      goal_handle->publish_feedback(feedback);
    }

    std::this_thread::sleep_for(exec_goal_time);
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }

    // goal is done
    goal_handle->succeed(result);
  }
};
}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__ACTION_CLIENT_MANAGER_HPP_
