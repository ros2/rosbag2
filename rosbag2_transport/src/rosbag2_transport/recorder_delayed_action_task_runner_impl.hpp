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

#ifndef ROSBAG2_TRANSPORT__RECORDER_DELAYED_ACTION_TASK_RUNNER_IMPL_HPP_
#define ROSBAG2_TRANSPORT__RECORDER_DELAYED_ACTION_TASK_RUNNER_IMPL_HPP_

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

class RecorderDelayedActionTaskRunnerImpl
{
public:
  explicit RecorderDelayedActionTaskRunnerImpl(rclcpp::Node * node)
  : node_(node)
  {}

  ~RecorderDelayedActionTaskRunnerImpl()
  {
    stop();
  }

  void start()
  {
    std::lock_guard<std::mutex> state_lock(start_stop_mutex_);
    if (is_running_.exchange(true)) {
      RCLCPP_ERROR(node_->get_logger(),
                  "RecorderDelayedActionTaskRunner is already running");
      return;
    }

    // Double-check thread state for safety
    if (thread_.joinable()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "RecorderDelayedActionTaskRunner thread is joinable but was "
        "marked as not running. Joining...");
      exit_.store(true);
      cv_.notify_all();
      thread_.join();
    }

    setup_clock_jump_callback();
    exit_.store(false);
    thread_ = std::thread(&RecorderDelayedActionTaskRunnerImpl::thread_main, this);
  }

  void stop()
  {
    std::lock_guard<std::mutex> state_lock(start_stop_mutex_);
    if (!is_running_.exchange(false)) {
      RCLCPP_ERROR(node_->get_logger(),
                  "RecorderDelayedActionTaskRunner is already stopped");
      return;
    }

    exit_.store(true);
    cv_.notify_all();
    if (thread_.joinable()) {
      thread_.join();
    }
    clock_jump_callback_.reset();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_ = decltype(queue_)();
    }
  }

  void schedule(
    const rclcpp::Time & scheduled_time,
    std::function<void()> action_task,
    const std::string & description)
  {
    ScheduledActionTask scheduled_action_task{
      scheduled_time,
      std::move(action_task),
      description,
      next_id_++
    };
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push(std::move(scheduled_action_task));
    }
    cv_.notify_all();
    RCLCPP_INFO(
      node_->get_logger(),
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

  bool is_ros_time_active() const
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

  void thread_main()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!exit_) {
      if (queue_.empty()) {
        cv_.wait(lock, [this]() {
            return exit_ || !queue_.empty();
        });
        continue;
      }

      const auto next_action = queue_.top();
      auto now = node_->now();
      if (next_action.deadline <= now) {
        queue_.pop();
        lock.unlock();
        execute_scheduled_action_task(next_action);
        lock.lock();
        continue;
      }

      const auto remaining = next_action.deadline - now;
      if (remaining.nanoseconds() <= 0) {
        continue;
      }

      if (is_ros_time_active()) {
        cv_.wait(
          lock,
          [this, &next_action]() {
            return exit_ || queue_.empty() ||
                   (queue_.top().sequence_id != next_action.sequence_id) ||
                   (queue_.top().deadline <= node_->now());
          });
      } else {
        const auto remaining_ns = std::chrono::nanoseconds(remaining.nanoseconds());
        const auto steady_deadline = std::chrono::steady_clock::now() + remaining_ns;
        cv_.wait_until(
          lock,
          steady_deadline,
          [this, &next_action]() {
            return exit_ || queue_.empty() ||
                   queue_.top().sequence_id != next_action.sequence_id;
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
        "Scheduled action_task '%s' failed due to an unknown error.",
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
  std::thread thread_;
  std::atomic_bool exit_{true};
  std::atomic_bool is_running_{false};
  std::mutex start_stop_mutex_;
  std::mutex mutex_;
  std::condition_variable cv_;
  uint64_t next_id_{0};
  std::priority_queue<ScheduledActionTask, std::vector<ScheduledActionTask>,
    ScheduledActionTaskComparator>
  queue_;
  rclcpp::JumpHandler::SharedPtr clock_jump_callback_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__RECORDER_DELAYED_ACTION_TASK_RUNNER_IMPL_HPP_
