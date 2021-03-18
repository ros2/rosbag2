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

#include "rosbag2_cpp/player_clock.hpp"
#include "rosbag2_cpp/types.hpp"

namespace
{

/**
 * Stores an exact time match between a system steady clock and the semantic play clock.
 * This is created whenever a factor changes such that a new base reference is needed
 * such as pause, resume, rate change, or jump
 */
struct TimeSync
{
  rosbag2_cpp::PlayerClock::PlayerTimePoint player_time;
  rosbag2_cpp::PlayerClock::RealTimePoint real_time;
};

template<typename T>
rcutils_duration_value_t duration_nanos(const T & duration)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

}  // namespace

namespace rosbag2_cpp
{

class PlayerClockImpl
{
public:
  PlayerClockImpl() = default;

  TimeSync reference;
  double rate = 1.0;
  PlayerClock::NowFunction now_fn;
  std::mutex mutex;
};

PlayerClock::PlayerClock(PlayerTimePoint starting_time, double rate, NowFunction now_fn)
: impl_(std::make_unique<PlayerClockImpl>())
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->now_fn = now_fn;
  impl_->reference.player_time = starting_time;
  impl_->reference.real_time = impl_->now_fn();
  impl_->rate = rate;
}

PlayerClock::~PlayerClock()
{}

PlayerClock::PlayerTimePoint PlayerClock::now() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  const auto real_diff = impl_->now_fn() - impl_->reference.real_time;
  const int64_t player_diff = duration_nanos(real_diff) * impl_->rate;
  return impl_->reference.player_time + player_diff;
}

bool PlayerClock::sleep_until(PlayerTimePoint until)
{
  RealTimePoint real_until;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const rcutils_duration_value_t diff_nanos =
      (until - impl_->reference.player_time) / impl_->rate;
    real_until = impl_->reference.real_time + std::chrono::nanoseconds(diff_nanos);
  }
  // TODO(emersonknapp) - when we have methods that can change timeflow during a sleep,
  // it will probably be better to use a condition_variable::wait_until
  std::this_thread::sleep_until(real_until);
  return now() >= until;
}

float PlayerClock::get_rate() const
{
  return impl_->rate;
}

}  // namespace rosbag2_cpp
