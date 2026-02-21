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

#ifndef ROSBAG2_TRANSPORT__DELAYED_ACTION_TASK_RUNNER_IMPL_HPP_
#define ROSBAG2_TRANSPORT__DELAYED_ACTION_TASK_RUNNER_IMPL_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace rosbag2_transport
{

class DelayedActionTaskRunnerImpl
{
public:
  explicit DelayedActionTaskRunnerImpl(rclcpp::Node * node)
  : node_(node)
  {}

  ~DelayedActionTaskRunnerImpl()
  {
    stop();
  }

  void start()
  {
    std::lock_guard<std::mutex> state_lock(start_stop_mutex_);
    if (is_running_.exchange(true)) {
      RCLCPP_WARN(node_->get_logger(), "DelayedActionTaskRunner is already running");
      return;
    }

    // Double-check thread state for safety
    if (runner_thread_.joinable()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "DelayedActionTaskRunner thread is joinable but was marked as not running. Joining...");
      exit_.store(true);
      cv_.notify_all();
      runner_thread_.join();
    }

    setup_clock_jump_callback();
    exit_.store(false);
    runner_thread_ = std::thread(&DelayedActionTaskRunnerImpl::runner_thread_main, this);
  }

  void stop()
  {
    std::lock_guard<std::mutex> state_lock(start_stop_mutex_);
    if (!is_running_.exchange(false)) {
      return;
    }

    exit_.store(true);
    cv_.notify_all();
    if (runner_thread_.joinable()) {
      runner_thread_.join();
    }
    clock_jump_callback_.reset();
    {
      std::lock_guard<std::mutex> lock(task_queue_mutex_);
      task_queue_ = decltype(task_queue_)();
    }
  }

  void schedule(
    const rclcpp::Time & scheduled_time,
    std::function<void()> action_task,
    const std::string & description)
  {
    {
      std::lock_guard<std::mutex> lock(task_queue_mutex_);
      ScheduledActionTask scheduled_action_task {
        scheduled_time,
        std::move(action_task),
        description,
        next_id_++
      };
      task_queue_.push(std::move(scheduled_action_task));
    }
    cv_.notify_all();
    RCLCPP_INFO(node_->get_logger(),
                "Scheduled '%s' for %.9f seconds",
                description.c_str(),
                scheduled_time.seconds());
  }

private:
  struct ScheduledActionTask
  {
    rclcpp::Time deadline;
    std::function<void()> callback;
    std::string description;
    uint64_t sequence_id;
  };

  struct ScheduledActionTaskComparator
  {
    bool operator()(const ScheduledActionTask & lhs, const ScheduledActionTask & rhs) const
    {
      if (lhs.deadline == rhs.deadline) {
        return lhs.sequence_id > rhs.sequence_id;
      }
      return lhs.deadline > rhs.deadline;
    }
  };

  [[nodiscard]] bool is_ros_time_active() const
  {
    bool ros_time_active = false;
    if (node_->get_clock()->get_clock_type() == RCL_ROS_TIME) {
      try {
        ros_time_active = node_->get_clock()->ros_time_is_active();
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          std::chrono::milliseconds(5000).count(),
          "Unable to determine if ROS time is active (%s). Falling back to steady wait.",
          e.what());
        ros_time_active = false;
      }
    }
    return ros_time_active;
  }

  void runner_thread_main()
  {
    std::unique_lock<std::mutex> lock(task_queue_mutex_);
    while (!exit_) {
      if (task_queue_.empty()) {
        cv_.wait(lock, [this]() {
            return exit_ || !task_queue_.empty();
        });
        continue;
      }

      const auto next_action_task = task_queue_.top();
      auto now = node_->now();
      if (next_action_task.deadline <= now) {
        task_queue_.pop();
        lock.unlock();
        execute_scheduled_action_task(next_action_task);
        lock.lock();
        continue;
      }

      const auto remaining_time = next_action_task.deadline - now;
      if (remaining_time.nanoseconds() <= 0) {
        continue;
      }

      if (is_ros_time_active()) {
        cv_.wait(
          lock,
          [this, &next_action_task]() {
            return exit_ || task_queue_.empty() ||
                   (task_queue_.top().sequence_id != next_action_task.sequence_id) ||
                   (task_queue_.top().deadline <= node_->now());
          });
      } else {
        const auto remaining_ns = std::chrono::nanoseconds(remaining_time.nanoseconds());
        const auto steady_deadline = std::chrono::steady_clock::now() + remaining_ns;
        cv_.wait_until(
          lock,
          steady_deadline,
          [this, &next_action_task]() {
            return exit_ || task_queue_.empty() ||
                   task_queue_.top().sequence_id != next_action_task.sequence_id;
          });
      }
    }
  }

  void execute_scheduled_action_task(const ScheduledActionTask & action_task)
  {
    if (exit_.load()) {
      return;
    }

    try {
      action_task.callback();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Scheduled action_task '%s' failed: %s", action_task.description.c_str(), e.what());
    } catch (...) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Scheduled action_task '%s' failed due to unknown error.",
        action_task.description.c_str());
    }
  }

  void setup_clock_jump_callback()
  {
    if (clock_jump_callback_ ||
      node_->get_clock()->get_clock_type() != RCL_ROS_TIME)
    {
      return;
    }

    rcl_jump_threshold_t threshold{};
    threshold.on_clock_change = true;
    threshold.min_backward.nanoseconds = -1;
    threshold.min_forward.nanoseconds = 1;
    clock_jump_callback_ = node_->get_clock()->create_jump_callback(
      nullptr,
      [this](const rcl_time_jump_t &) noexcept {
        cv_.notify_all();
      },
      threshold);
  }

  rclcpp::Node * node_;
  std::thread runner_thread_;
  std::atomic_bool exit_{true};
  std::atomic_bool is_running_{false};
  std::mutex start_stop_mutex_;
  std::mutex task_queue_mutex_;
  std::condition_variable cv_;
  uint64_t next_id_{0};
  std::priority_queue<
    ScheduledActionTask, std::vector<ScheduledActionTask>, ScheduledActionTaskComparator>
  task_queue_;
  rclcpp::JumpHandler::SharedPtr clock_jump_callback_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__DELAYED_ACTION_TASK_RUNNER_IMPL_HPP_
