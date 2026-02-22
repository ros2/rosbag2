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
#include "rosbag2_transport/visibility_control.hpp"

#ifdef _WIN32
#  pragma warning(push)
// Suppress warning "rosbag2_transport::DelayedActionTaskRunner::pimpl_': class
// 'std::unique_ptr>' needs to have dll-interface to be used by clients of class
// 'rosbag2_transport::DelayedActionTaskRunner'"
// Justification:
// 1. We never inline code in the header that actually calls methods on
// DelayedActionTaskRunnerImpl.
// 2. While the `DelayedActionTaskRunnerImpl` is defined in the
// `delayed_action_task_runner_impl.hpp` file, we include it only in the
// `delayed_action_task_runner.cpp` file, and it does not leak into the external API.
// 3. The pimpl design pattern imply that implementation details are hidden and shouldn't be
// exposed with the dll-interface.
#  pragma warning(disable:4251)
#endif

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace rosbag2_transport
{
class DelayedActionTaskRunnerImpl;
/// @brief Background helper that runs actions at specific ROS times.
/// @details Uses a dedicated thread with a priority queue to execute callbacks once the
/// node clock reaches a scheduled timestamp. Keeps waitset/service threads free from long sleeps.
class ROSBAG2_TRANSPORT_PUBLIC DelayedActionTaskRunner
{
public:
  /// @brief Construct a runner bound to the provided node.
  /// @param node Node used for logging and to query current time.
  explicit DelayedActionTaskRunner(rclcpp::Node * node);

  /// @brief Default destructor.
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

#ifdef _WIN32
#  pragma warning(pop)
#endif

#endif  // ROSBAG2_TRANSPORT__DELAYED_ACTION_TASK_RUNNER_HPP_
