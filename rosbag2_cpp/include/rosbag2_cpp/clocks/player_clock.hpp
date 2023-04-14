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

#ifndef ROSBAG2_CPP__CLOCKS__PLAYER_CLOCK_HPP_
#define ROSBAG2_CPP__CLOCKS__PLAYER_CLOCK_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rcutils/time.h"
#include "rosbag2_cpp/visibility_control.hpp"

namespace rosbag2_cpp
{

/**
 * Virtual interface used to drive the timing of bag playback.
 * This clock should be used to query times and sleep between message playing,
 * so that the complexity involved around time control and time sources
 * is encapsulated in this one place.
 */
class PlayerClock
{
public:
  /**
   * Type representing an arbitrary steady time, used to measure real-time durations
   * This type is never exposed by the PlayerClock - it is only used as input to the PlayerClock.
   */
  typedef std::function<std::chrono::steady_clock::time_point()> NowFunction;

  ROSBAG2_CPP_PUBLIC
  virtual ~PlayerClock() = default;

  /**
   * Calculate and return current rcutils_time_point_value_t based on starting time, playback rate, pause state.
   */
  ROSBAG2_CPP_PUBLIC
  virtual rcutils_time_point_value_t now() const = 0;

  /**
   * @brief Convert an arbitrary ROSTime to a SteadyTime, based on the current reference snapshot.
   * @param ros_time - time point in ROSTime
   * @return time point in steady clock i.e. std::chrono::steady_clock
   */
  ROSBAG2_CPP_PUBLIC
  virtual std::chrono::steady_clock::time_point
  ros_to_steady(rcutils_time_point_value_t ros_time) const = 0;

  /**
   * Try to sleep (non-busy) the current thread until the provided time is reached - according to this Clock
   *
   * Return true if the time has been reached, false if it was not successfully reached after sleeping
   * for the appropriate duration.
   * The user should not take action based on this sleep until it returns true.
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool sleep_until(rcutils_time_point_value_t until) = 0;

  /**
   * \sa sleep_until
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool sleep_until(rclcpp::Time time) = 0;

  /**
   * Change the rate of the flow of time for the clock.
   * \param rate new rate of clock playback
   * \bool false if rate is invalid for the clock implementation
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool set_rate(double rate) = 0;

  /**
   * \return the current playback rate.
   */
  ROSBAG2_CPP_PUBLIC
  virtual double get_rate() const = 0;

  /**
   * Stop the flow of time of the clock.
   * If this changes the pause state, this will wake any waiting `sleep_until`
   */
  ROSBAG2_CPP_PUBLIC
  virtual void pause() = 0;

  /**
   * Start the flow of time of the clock
   * If this changes the pause state, this will wake any waiting `sleep_until`
   */
  ROSBAG2_CPP_PUBLIC
  virtual void resume() = 0;

  /**
   * Return whether the clock is currently paused.
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool is_paused() const = 0;

  /**
   * \brief Change the current ROS time to an arbitrary time.
   * \note This will wake any waiting `sleep_until` and trigger any registered JumpHandler
   * callbacks.
   * \param time_point Time point in ROS playback timeline.
   */
  ROSBAG2_CPP_PUBLIC
  virtual void jump(rcutils_time_point_value_t time_point) = 0;

  /**
   * \sa jump
   */
  ROSBAG2_CPP_PUBLIC
  virtual void jump(rclcpp::Time time) = 0;

  /// Add a callback to invoke if the jump threshold is exceeded.
  /// \sa rclcpp::Clock::create_jump_callback
  ROSBAG2_CPP_PUBLIC
  virtual rclcpp::JumpHandler::SharedPtr create_jump_callback(
    rclcpp::JumpHandler::pre_callback_t pre_callback,
    rclcpp::JumpHandler::post_callback_t post_callback,
    const rcl_jump_threshold_t & threshold) = 0;
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CLOCKS__PLAYER_CLOCK_HPP_
