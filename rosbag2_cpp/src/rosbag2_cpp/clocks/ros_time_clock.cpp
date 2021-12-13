// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rcpputils/thread_safety_annotations.hpp"
#include "rosbag2_cpp/clocks/ros_time_clock.hpp"
#include "rosbag2_cpp/types.hpp"
#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{

RosTimeClock::RosTimeClock(rclcpp::Clock::SharedPtr clock)
: clock_(clock)
{
}

RosTimeClock::~RosTimeClock()
{}

rcutils_time_point_value_t RosTimeClock::now() const
{
  return clock_->now().nanoseconds();
}

bool RosTimeClock::sleep_until(rcutils_time_point_value_t until)
{
  return sleep_until(rclcpp::Time(until, RCL_ROS_TIME));
}

bool RosTimeClock::sleep_until(rclcpp::Time until)
{
  std::mutex mutex;
  std::condition_variable cv;
  bool time_source_changed;

  // Watch the clock for any time change at all.
  rcl_jump_threshold_t thresh;
  thresh.on_clock_change = true;
  thresh.min_backward.nanoseconds = -1;
  thresh.min_forward.nanoseconds = 1;
  auto clock_handler = create_jump_callback(
    nullptr,
    [&cv, &time_source_changed](const rcl_time_jump_t & jump) {
      if (jump.clock_change != RCL_ROS_TIME_NO_CHANGE) {
        time_source_changed = true;
      }
      cv.notify_all();
    },
    thresh);

  // Wake up periodically so that control could be given back to the program if desired.
  std::unique_lock lock(mutex);
  ROSBAG2_CPP_LOG_WARN("Sleeping until %ld", until.nanoseconds());
  while (rclcpp::ok() && clock_->now() < until) {
    cv.wait_for(lock, std::chrono::milliseconds(10));
    ROSBAG2_CPP_LOG_WARN("Done one sleep, clock time is now %ld", clock_->now().nanoseconds());
  }
  if (!rclcpp::ok()) {
    return false;
  }

  return clock_->now() >= until;
}

rclcpp::JumpHandler::SharedPtr RosTimeClock::create_jump_callback(
  rclcpp::JumpHandler::pre_callback_t pre_callback,
  rclcpp::JumpHandler::post_callback_t post_callback,
  const rcl_jump_threshold_t & threshold)
{
  return clock_->create_jump_callback(pre_callback, post_callback, threshold);
}

}  // namespace rosbag2_cpp
