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

#include <memory>
#include <mutex>
#include <thread>

#include "rcpputils/thread_safety_annotations.hpp"
#include "rosbag2_cpp/player_clock.hpp"
#include "rosbag2_cpp/types.hpp"

namespace rosbag2_cpp
{

class PlayerClockImpl
{
public:
  /**
   * Stores an exact time match between a system steady clock and the playback ROS clock.
   * This snapshot is taken whenever a factor changes such that a new reference is needed,
   * such as pause, resume, rate change, or jump
   */
  struct TimeReference
  {
    PlayerClock::ROSTimePoint ros;
    PlayerClock::SteadyTimePoint steady;
  };

  PlayerClockImpl() = default;
  virtual ~PlayerClockImpl() = default;

  template<typename T>
  rcutils_duration_value_t duration_nanos(const T & duration)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  }

  PlayerClock::ROSTimePoint steady_to_ros(PlayerClock::SteadyTimePoint steady_time)
  {
    return reference.ros + (rate * duration_nanos(steady_time - reference.steady));
  }

  PlayerClock::SteadyTimePoint ros_to_steady(PlayerClock::ROSTimePoint ros_time)
  {
    const rcutils_duration_value_t diff_nanos = (ros_time - reference.ros) / rate;
    return reference.steady + std::chrono::nanoseconds(diff_nanos);
  }

  double rate = 1.0;
  PlayerClock::NowFunction now_fn;
  std::mutex mutex;
  TimeReference reference RCPPUTILS_TSA_GUARDED_BY(mutex);
};

PlayerClock::PlayerClock(ROSTimePoint starting_time, double rate, NowFunction now_fn)
: impl_(std::make_unique<PlayerClockImpl>())
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->now_fn = now_fn;
  impl_->reference.ros = starting_time;
  impl_->reference.steady = impl_->now_fn();
  impl_->rate = rate;
}

PlayerClock::~PlayerClock()
{}

PlayerClock::ROSTimePoint PlayerClock::now() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->steady_to_ros(impl_->now_fn());
}

bool PlayerClock::sleep_until(ROSTimePoint until)
{
  SteadyTimePoint steady_until;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    steady_until = impl_->ros_to_steady(until);
  }
  // TODO(emersonknapp) - when we have methods that can change timeflow during a sleep,
  // it will probably be better to use a condition_variable::wait_until
  std::this_thread::sleep_until(steady_until);
  return now() >= until;
}

double PlayerClock::get_rate() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->rate;
}

}  // namespace rosbag2_cpp
