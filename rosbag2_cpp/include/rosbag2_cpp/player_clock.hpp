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
 * Clock used drive the timing of bag playback.
 * Takes an input ROS clock and manages an offset to it, providing
 * This clock should be used to query times and sleep between message playing,
 * so that the logic around translateing underlying time sources is encapsulated in this one place.
 */
class PlayerClockImpl;
class PlayerClock
{
public:
  /**
   * Constructor.
   *
   * \param starting_time: provides an initial offset for managing time
   *    This will likely be the timestamp of the first message in the bag
   * \param now_fn: Function used to get the current steady time
   *   defaults to std::chrono::steady_clock::now
   *   Used to control for unit testing, or for specialized needs
   * \param sleep_check_period: Amount of time to sleep in `sleep_until` when the clock
   *   is paused. Allows the caller to spin at a defined rate while receiving `false`
   * \param paused: Start the clock paused
   */
  ROSBAG2_CPP_PUBLIC
  PlayerClock(
    rcutils_time_point_value_t starting_time,
    rclcpp::Clock::SharedPtr clock,
    std::chrono::milliseconds sleep_checkin_period = std::chrono::milliseconds{100},
    bool start_paused = false);

  ROSBAG2_CPP_PUBLIC
  virtual ~PlayerClock();

  /**
   * Calculate and return current rcutils_time_point_value_t based on starting time, playback rate, pause state.
   */
  ROSBAG2_CPP_PUBLIC
  virtual rcutils_time_point_value_t now() const;

  /**
   * Try to sleep (non-busy) the current thread until the provided time is reached - according to this Clock
   *
   * Return true if the time has been reached, false if it was not successfully reached after sleeping
   * for the appropriate duration.
   * The user should not take action based on this sleep until it returns true.
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool sleep_until(rcutils_time_point_value_t until);

  /**
   * Change the rate of the flow of time for the clock.
   * \param rate new rate of clock playback
   * \bool false if rate is invalid for the clock implementation
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool set_rate(double rate);

  /**
   * \return the current playback rate.
   */
  ROSBAG2_CPP_PUBLIC
  virtual double get_rate() const;

  /**
   * Stop the flow of time of the clock.
   * If this changes the pause state, this will wake any waiting `sleep_until`
   */
  ROSBAG2_CPP_PUBLIC
  virtual void pause();

  /**
   * Start the flow of time of the clock
   * If this changes the pause state, this will wake any waiting `sleep_until`
   */
  ROSBAG2_CPP_PUBLIC
  virtual void resume();

  /**
   * Return whether the clock is currently paused.
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool is_paused() const;

  /**
   * \brief Change the internal offset used to calculate "now".
   * Each clock implementation has an underlying source of truth for time, and maintains
   * some offset to associate that with a bag time.
   * \note This will wake any waiting `sleep_until` and trigger any registered JumpHandler
   * callbacks.
   * \param time_point Current time point in bag's reference frame, to match with internal now.
   */
  ROSBAG2_CPP_PUBLIC
  virtual void override_offset(rcutils_time_point_value_t);

private:
  std::unique_ptr<PlayerClockImpl> impl_;
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CLOCKS__PLAYER_CLOCK_HPP_
