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

#ifndef ROSBAG2_CPP__CLOCKS__SIM_TIME_CLOCK_HPP_
#define ROSBAG2_CPP__CLOCKS__SIM_TIME_CLOCK_HPP_

#include <memory>

#include "rosbag2_cpp/clocks/player_clock.hpp"

namespace rosbag2_cpp
{

class SimTimeClockImpl;
class SimTimeClock : public PlayerClock
{
public:
  ROSBAG2_CPP_PUBLIC
  SimTimeClock();

  ROSBAG2_CPP_PUBLIC
  virtual ~SimTimeClock();

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

  /**
   * No-op. Unsupported for this clock.
   */
  ROSBAG2_CPP_PUBLIC
  void set_rate(double rate) override;

  /**
   * Return a best-guess of the current rate of time.
   */
  ROSBAG2_CPP_PUBLIC
  double get_rate() const override;

  /**
   * No-op. Unsupported for this clock.
   */
  ROSBAG2_CPP_PUBLIC
  void pause() override;

  /**
   * No-op. Unsupported for this clock.
   */
  ROSBAG2_CPP_PUBLIC
  void resume() override;

  /**
   * Return best-guess as to whether sim-time is currently moving.
   */
  ROSBAG2_CPP_PUBLIC
  bool is_paused() const override;

private:
  std::unique_ptr<SimTimeClockImpl> impl_;
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CLOCKS__SIM_TIME_CLOCK_HPP_
