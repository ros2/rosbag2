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

#ifndef ROSBAG2_CPP__PLAYER_CLOCK_HPP_
#define ROSBAG2_CPP__PLAYER_CLOCK_HPP_

#include <chrono>
#include <functional>
#include <memory>

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
  // Type representing the current time as according to the playback
  typedef rcutils_time_point_value_t ROSTimePoint;
  /**
   * Type representing an arbitrary steady time, used to measure real-time durations
   * This type is never exposed by the PlayerClock - it is only used as input to the PlayerClock.
   */
  typedef std::chrono::time_point<std::chrono::steady_clock> SteadyTimePoint;
  typedef std::function<SteadyTimePoint()> NowFunction;

  ROSBAG2_CPP_PUBLIC
  virtual ~PlayerClock() = default;

  /**
   * Calculate and return current ROSTimePoint based on starting time, playback rate, pause state.
   */
  ROSBAG2_CPP_PUBLIC
  virtual ROSTimePoint now() const = 0;

  /**
   * Try to sleep (non-busy) the current thread until the provided time is reached - according to this Clock
   *
   * Return true if the time has been reached, false if it was not successfully reached after sleeping
   * for the appropriate duration.
   * The user should not take action based on this sleep until it returns true.
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool sleep_until(ROSTimePoint until) = 0;

  /**
   * Return the current playback rate.
   */
  ROSBAG2_CPP_PUBLIC
  virtual double get_rate() const = 0;
};

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
    ROSTimePoint starting_time,
    double rate = 1.0,
    NowFunction now_fn = std::chrono::steady_clock::now);

  ROSBAG2_CPP_PUBLIC
  virtual ~TimeControllerClock();

  /**
   * Calculate and return current ROSTimePoint based on starting time, playback rate, pause state.
   */
  ROSBAG2_CPP_PUBLIC
  ROSTimePoint now() const override;

  /**
   * Try to sleep (non-busy) the current thread until the provided time is reached - according to this Clock
   *
   * Return true if the time has been reached, false if it was not successfully reached after sleeping
   * for the appropriate duration.
   * The user should not take action based on this sleep until it returns true.
   */
  ROSBAG2_CPP_PUBLIC
  bool sleep_until(ROSTimePoint until) override;

  /**
   * Return the current playback rate.
   */
  ROSBAG2_CPP_PUBLIC
  double get_rate() const override;

private:
  std::unique_ptr<TimeControllerClockImpl> impl_;
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__PLAYER_CLOCK_HPP_
