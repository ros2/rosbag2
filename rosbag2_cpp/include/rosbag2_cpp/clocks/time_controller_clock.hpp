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

#ifndef ROSBAG2_CPP__CLOCKS__TIME_CONTROLLER_CLOCK_HPP_
#define ROSBAG2_CPP__CLOCKS__TIME_CONTROLLER_CLOCK_HPP_

#include <chrono>
#include <memory>

#include "rosbag2_cpp/clocks/player_clock.hpp"

namespace rosbag2_cpp
{

/**
 * Version of the PlayerClock interface that has control over time.
 * It does not listen to any external ROS Time Source and can optionally publish /clock
 */
class TimeControllerClockImpl;
class TimeControllerClock : public PlayerClock
{
public:
  /**
   * Constructor.
   *
   * \param starting_time: provides an initial offset for managing time
   *    This will likely be the timestamp of the first message in the bag
   * \param rate: Rate of playback, a unit-free ratio. 1.0 is real-time
   * \param now_fn: Function used to get the current steady time
   *   defaults to std::chrono::steady_clock::now
   *   Used to control for unit testing, or for specialized needs
   * \throws std::runtime_error if rate is <= 0
   */
  ROSBAG2_CPP_PUBLIC
  TimeControllerClock(
    rcutils_time_point_value_t starting_time,
    double rate = 1.0,
    NowFunction now_fn = std::chrono::steady_clock::now,
    std::chrono::milliseconds sleep_time_while_paused = std::chrono::milliseconds{100});

  ROSBAG2_CPP_PUBLIC
  virtual ~TimeControllerClock();

  /**
   * Calculate and return current rcutils_time_point_value_t based on starting time, playback rate, pause state.
   */
  ROSBAG2_CPP_PUBLIC
  rcutils_time_point_value_t now() const override;

  /**
   * Try to sleep (non-busy) the current thread until the provided time is reached - according to this Clock
   *
   * Return true if the time has been reached, false if it was not successfully reached after sleeping
   * for the appropriate duration.
   * The user should not take action based on this sleep until it returns true.
   */
  ROSBAG2_CPP_PUBLIC
  bool sleep_until(rcutils_time_point_value_t until) override;

  /// \brief Adjust internal clock to the specified timestamp
  /// \details It will change the current internally maintained offset so that next published time
  /// is different.
  /// \note Will trigger any registered JumpHandler callbacks.
  /// \param time_point Time point in ROS playback timeline.
  ROSBAG2_CPP_PUBLIC
  void jump(rcutils_time_point_value_t time_point) override;

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
  PlayerClock::JumpHandler::SharedPtr create_jump_handler(
    const JumpHandler::pre_callback_t & pre_callback,
    const JumpHandler::post_callback_t & post_callback,
    const rcl_jump_threshold_t & threshold) override;

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
  void add_jump_calbacks(PlayerClock::JumpHandler::SharedPtr handler) override;

  /// \brief remove jump callbacks from processing list.
  /// \param handler Shared pointer to the JumpHandler object returned from create_jump_handler()
  ROSBAG2_CPP_PUBLIC
  void remove_jump_callbacks(PlayerClock::JumpHandler::SharedPtr handler) override;

  /**
   * Return the current playback rate.
   */
  ROSBAG2_CPP_PUBLIC
  double get_rate() const override;

  /**
   * Stop the flow of time of the clock.
   * If this changes the pause state, this will wake any waiting `sleep_until`
   */
  ROSBAG2_CPP_PUBLIC
  void pause() override;

  /**
   * Start the flow of time of the clock
   * If this changes the pause state, this will wake any waiting `sleep_until`
   */
  ROSBAG2_CPP_PUBLIC
  void resume() override;

  /**
   * Return whether the clock is currently paused.
   */
  ROSBAG2_CPP_PUBLIC
  bool is_paused() const override;

private:
  std::unique_ptr<TimeControllerClockImpl> impl_;
};


}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CLOCKS__TIME_CONTROLLER_CLOCK_HPP_
