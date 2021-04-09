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

#include <chrono>
#include <functional>
#include <memory>

#include "rcutils/time.h"
#include "rclcpp/clock.hpp"
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

  using JumpHandler = rclcpp::JumpHandler;

  ROSBAG2_CPP_PUBLIC
  virtual ~PlayerClock() = default;

  /**
   * Calculate and return current rcutils_time_point_value_t based on starting time, playback rate, pause state.
   */
  ROSBAG2_CPP_PUBLIC
  virtual rcutils_time_point_value_t now() const = 0;

  /**
   * Try to sleep (non-busy) the current thread until the provided time is reached - according to this Clock
   *
   * Return true if the time has been reached, false if it was not successfully reached after sleeping
   * for the appropriate duration.
   * The user should not take action based on this sleep until it returns true.
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool sleep_until(rcutils_time_point_value_t until) = 0;

  /// \brief Adjust internal clock to the specified timestamp
  /// \details It will change the current internally maintained offset so that next published time
  /// is different.
  /// \note Will trigger any registered JumpHandler callbacks.
  /// \param time_point Time point in ROS playback timeline.
  ROSBAG2_CPP_PUBLIC
  virtual void jump(rcutils_time_point_value_t time_point) = 0;

  /// \brief Creates JumpHandler object with callbacks for jump operation.
  /// \param pre_callback Pre-time jump callback. Must be non-throwing.
  /// \param post_callback Post-time jump callback. Must be non-throwing.
  /// \param threshold Callbacks will be triggered if the time jump is greater then the threshold.
  /// \note These callback functions must remain valid as long as the returned shared pointer is
  /// valid.
  /// \return Shared pointer to the newly created JumpHandler object.
  /// \throws std::bad_alloc if the allocation of the JumpHandler fails.
  /// \warning If data members of the JumpHandler object will be changed out of PlayerClock class,
  /// it will cause undefined behaviour.
  ROSBAG2_CPP_PUBLIC
  virtual PlayerClock::JumpHandler::SharedPtr create_jump_handler(
    const JumpHandler::pre_callback_t & pre_callback,
    const JumpHandler::post_callback_t & post_callback,
    const rcl_jump_threshold_t & threshold) = 0;

  /// \brief Add callbacks to be called when a time jump exceeds a threshold.
  /// \details Callbacks specified in JumpHandler object will be called in two cases:
  ///    1. use_sim_time is true: if the external time source jumps back in time, or forward
  /// farther than the threshold.
  ///    2. use_sim_time is false: if jump(time_point) is called and time jumps back or forward
  /// farther than the threshold.
  /// \param handler Shared pointer to the JumpHandler object returned from create_jump_handler()
  /// \throws std::invalid argument if jump threshold has invalid value.
  /// \note Pair of pre_callback and post_callback shall be unique for each JumpHandler object.
  ROSBAG2_CPP_PUBLIC
  virtual void add_jump_calbacks(PlayerClock::JumpHandler::SharedPtr handler) = 0;

  /// \brief remove jump callbacks from processing list.
  /// \param handler Shared pointer to the JumpHandler object returned from create_jump_handler()
  ROSBAG2_CPP_PUBLIC
  virtual void remove_jump_callbacks(PlayerClock::JumpHandler::SharedPtr handler) = 0;

  /**
   * Return the current playback rate.
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
};

/// \brief Equal operator for PlayerClock::JumpHandler class.
/// \param left Left side operand for equal comparison operation.
/// \param right Right side operand for equal comparison operation.
/// \return true if corresponding object's callbacks pointing to the same functions,
/// otherwise false.
ROSBAG2_CPP_PUBLIC
bool operator==(const PlayerClock::JumpHandler & left, const PlayerClock::JumpHandler & right)
{
  if (&(left.pre_callback) == &(right.pre_callback) &&
    &(left.post_callback) == &(right.post_callback))
  {
    return true;
  } else {
    return false;
  }
}

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CLOCKS__PLAYER_CLOCK_HPP_
