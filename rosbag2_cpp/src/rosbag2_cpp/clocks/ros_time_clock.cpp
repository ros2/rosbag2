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
: clock_(clock),
  offset_(0)
{
}

RosTimeClock::~RosTimeClock()
{}

rcutils_time_point_value_t RosTimeClock::now() const
{
  return (clock_->now() + rclcpp::Duration::from_nanoseconds(offset_)).nanoseconds();
}

bool RosTimeClock::sleep_until(rcutils_time_point_value_t until)
{
  return sleep_until(rclcpp::Time(until, RCL_ROS_TIME));
}

bool RosTimeClock::sleep_until(rclcpp::Time until)
{
  return clock_->sleep_until(until + rclcpp::Duration::from_nanoseconds(offset_));
}

void RosTimeClock::override_offset(rcutils_time_point_value_t bag_time)
{
  auto current = clock_->now();
  RCUTILS_LOG_ERROR("Making offset from clock's %ld to bag's %ld",
    current.nanoseconds(), bag_time);
  offset_ = current.nanoseconds() - bag_time;
  RCUTILS_LOG_ERROR("New offset value is %ld", offset_);
}

rclcpp::JumpHandler::SharedPtr RosTimeClock::create_jump_callback(
  rclcpp::JumpHandler::pre_callback_t pre_callback,
  rclcpp::JumpHandler::post_callback_t post_callback,
  const rcl_jump_threshold_t & threshold)
{
  return clock_->create_jump_callback(pre_callback, post_callback, threshold);
}

}  // namespace rosbag2_cpp
