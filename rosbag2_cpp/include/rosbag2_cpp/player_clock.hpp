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

namespace rosbag2_cpp
{
class PlayerClockImpl;

/**
 * Used to control the timing of bag playback.
 * This clock should be used to query times and sleep between message playing,
 * so that the complexity involved around time control and time sources
 * is encapsulated in this one place.
 */
class PlayerClock
{
public:
  typedef rcutils_time_point_value_t PlayerTimePoint;
  typedef std::chrono::time_point<std::chrono::steady_clock> RealTimePoint;
  typedef std::function<RealTimePoint()> NowFunction;

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
  PlayerClock(
    PlayerTimePoint starting_time,
    double rate = 1.0,
    NowFunction now_fn = std::chrono::steady_clock::now);

  virtual ~PlayerClock();

  /**
   * Calculate and return current PlayerTimePoint based on starting time, playback rate, pause state.
   */
  PlayerTimePoint now() const;

  /**
   * Try to sleep (non-busy) the current thread until the provided time is reached - according to this Clock
   *
   * Return true if the time has been reached, false if it was not successfully reached after sleeping
   * for the appropriate duration.
   * The user should not take action based on this sleep until it returns true.
   */
  bool sleep_until(PlayerTimePoint until);

  /**
   * Return the current playback rate.
   */
  double get_rate() const;

private:
  std::unique_ptr<PlayerClockImpl> impl_;
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__PLAYER_CLOCK_HPP_
