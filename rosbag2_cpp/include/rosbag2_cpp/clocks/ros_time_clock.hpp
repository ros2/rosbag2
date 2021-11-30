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

#ifndef ROSBAG2_CPP__CLOCKS__ROS_TIME_CLOCK_HPP_
#define ROSBAG2_CPP__CLOCKS__ROS_TIME_CLOCK_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/clock.hpp"

#include "rosbag2_cpp/clocks/player_clock.hpp"

namespace rosbag2_cpp
{

/// Implementation of PlayerClock that uses ROStime and cannot control time itself.
/// All time control methods such as set_rate and seek will not do anything.
/// If no /clock topic is publishing, time will not move forward.
class RosTimeClock : public PlayerClock
{
public:
  ROSBAG2_CPP_PUBLIC
  RosTimeClock();

  ROSBAG2_CPP_PUBLIC
  virtual ~RosTimeClock();

  /// Returns the latest /clock sample received
  ROSBAG2_CPP_PUBLIC
  rcutils_time_point_value_t now() const override;

  /// Try to sleep (non-busy) the current thread until the provided time is reached according to /clock
  /// \return true if time has been reached, false if it was not successfully reached
  ROSBAG2_CPP_PUBLIC
  bool sleep_until(rcutils_time_point_value_t until) override;

  ROSBAG2_CPP_PUBLIC
  bool sleep_until(rclcpp::Time until) override;

  /// No-op
  ROSBAG2_CPP_PUBLIC
  bool set_rate(double /*rate*/) override
  {
    return false;
  }

  /// No-op
  ROSBAG2_CPP_PUBLIC
  double get_rate() const override
  {
    return 1.0;
  }

  /// No-op
  ROSBAG2_CPP_PUBLIC
  void pause() override {}

  /// No-op
  ROSBAG2_CPP_PUBLIC
  void resume() override {}

  /// No-op
  ROSBAG2_CPP_PUBLIC
  bool is_paused() const override
  {
    return false;
  }

  /// No-op
  ROSBAG2_CPP_PUBLIC
  void jump(rcutils_time_point_value_t /*ros_time*/) override {}

  /// No-op
  ROSBAG2_CPP_PUBLIC
  void jump(rclcpp::Time /*ros_time*/) override {}

  /// Add a callback to invoke if the jump threshold is exceeded.
  /// \sa rclcpp::Clock::create_jump_callback
  ROSBAG2_CPP_PUBLIC
  rclcpp::JumpHandler::SharedPtr create_jump_callback(
    rclcpp::JumpHandler::pre_callback_t pre_callback,
    rclcpp::JumpHandler::post_callback_t post_callback,
    const rcl_jump_threshold_t & threshold) override;

private:
  std::unique_ptr<rclcpp::Clock> clock_;
};
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CLOCKS__ROS_TIME_CLOCK_HPP_
