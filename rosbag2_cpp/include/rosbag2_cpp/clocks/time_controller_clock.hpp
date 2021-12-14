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
 * It does not listen to any ROS time source, instead using an internal steady time.
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
   * \param now_fn: Function used to get the current steady time
   *   defaults to std::chrono::steady_clock::now
   *   Used to control for unit testing, or for specialized needs
   * \param sleep_time_while_paused: Amount of time to sleep in `sleep_until` when the clock
   *   is paused. Allows the caller to spin at a defined rate while receiving `false`
   * \param paused: Start the clock paused
   */
  ROSBAG2_CPP_PUBLIC
  TimeControllerClock(
    rcutils_time_point_value_t starting_time,
    NowFunction now_fn = std::chrono::steady_clock::now,
    std::chrono::milliseconds sleep_time_while_paused = std::chrono::milliseconds{100},
    bool start_paused = false);

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

  ROSBAG2_CPP_PUBLIC
  bool sleep_until(rclcpp::Time until) override;

  /**
   * Change the rate of the flow of time for the clock.
   *
   * To stop time, \sa pause.
   * It is not currently valid behavior to move time backwards.
   * \param rate The new rate of time
   * \return false if rate <= 0, true otherwise.
   */
  ROSBAG2_CPP_PUBLIC
  bool set_rate(double rate) override;

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

  /**
   * Change the current time to an arbitrary time.
   * \note This will wake any waiting `sleep_until`, and may trigger jump callbacks.
   *
   * \param ros_time: The new bag time to use as the basis for "now()"
   */
  ROSBAG2_CPP_PUBLIC
  void override_offset(rcutils_time_point_value_t bag_time) override;

  /// Since a jump can only occur via a `jump` call by the owner of this Clock,
  /// jump callbacks are not handled in this clock.
  /// It is expected that the caller handles jumps in their calling code.
  /// \return nullptr
  ROSBAG2_CPP_PUBLIC
  rclcpp::JumpHandler::SharedPtr create_jump_callback(
    rclcpp::JumpHandler::pre_callback_t pre_callback,
    rclcpp::JumpHandler::post_callback_t post_callback,
    const rcl_jump_threshold_t & threshold) override;

private:
  std::unique_ptr<TimeControllerClockImpl> impl_;
};


}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CLOCKS__TIME_CONTROLLER_CLOCK_HPP_
