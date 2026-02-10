// Copyright 2025 Apex.AI, Inc.
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

#ifndef ROSBAG2_TRANSPORT__DELAYED_ACTION_TASK_RUNNER_HPP_
#define ROSBAG2_TRANSPORT__DELAYED_ACTION_TASK_RUNNER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/time.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace rosbag2_transport
{
class DelayedActionTaskRunnerImpl;
/// @brief Background helper that runs actions at specific ROS times.
/// @details Uses a dedicated thread with a priority queue to execute callbacks once the
/// recorder node clock reaches a scheduled timestamp. Keeps the recorder waitset/service
/// threads free from long sleeps.
class DelayedActionTaskRunner
{
public:
  /// @brief Construct a runner bound to the provided node.
  /// @param node Node used for logging and to query current time.
  explicit DelayedActionTaskRunner(rclcpp::Node * node);
  ~DelayedActionTaskRunner();

  /// @brief Start the background thread.
  /// @note Subsequent calls are no-ops while the thread is running.
  void start();

  /// @brief Stop the background thread and clear outstanding actions.
  /// @note Safe to call multiple times.
  void stop();

  /// @brief Schedule an action at a specific ROS time.
  /// @param scheduled_time Absolute time when the action should fire.
  /// @param action Callback executed when scheduled time is reached.
  /// @param description Human-readable label used for debug logging.
  void schedule(
    const rclcpp::Time & scheduled_time,
    std::function<void()> action,
    const std::string & description);

private:
  std::unique_ptr<DelayedActionTaskRunnerImpl> pimpl_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__DELAYED_ACTION_TASK_RUNNER_HPP_
