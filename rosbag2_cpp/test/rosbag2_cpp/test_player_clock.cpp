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

#include <gmock/gmock.h>

#include "rosbag2_cpp/player_clock.hpp"

using namespace testing;  // NOLINT
using SteadyTimePoint = rosbag2_cpp::PlayerClock::SteadyTimePoint;
using ROSTimePoint = rosbag2_cpp::PlayerClock::ROSTimePoint;
using NowFunction = rosbag2_cpp::PlayerClock::NowFunction;

ROSTimePoint as_nanos(const SteadyTimePoint & time)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

TEST(PlayerClock, steadytime_precision)
{
  SteadyTimePoint return_time;
  NowFunction now_fn = [&return_time]() {
      return return_time;
    };
  rosbag2_cpp::PlayerClock pclock(0, 1.0, now_fn);

  const SteadyTimePoint begin_time(std::chrono::seconds(0));
  return_time = begin_time;
  EXPECT_EQ(pclock.now(), as_nanos(begin_time));

  const SteadyTimePoint ten_seconds(std::chrono::seconds(10));
  return_time = ten_seconds;
  EXPECT_EQ(pclock.now(), as_nanos(ten_seconds));

  // NOTE: this would have already lost precision at 100 seconds if we were multiplying by float
  const SteadyTimePoint hundred_seconds(std::chrono::seconds(100));
  return_time = hundred_seconds;
  EXPECT_EQ(pclock.now(), as_nanos(hundred_seconds));

  const int64_t near_limit_nanos = 1LL << 61;
  const auto near_limit_time = SteadyTimePoint(std::chrono::nanoseconds(near_limit_nanos));
  return_time = near_limit_time;
  EXPECT_EQ(pclock.now(), near_limit_nanos);

  // Expect to lose exact equality at this point due to precision limit of double*int mult
  const int64_t over_limit_nanos = (1LL << 61) + 1LL;
  const auto over_limit_time = SteadyTimePoint(std::chrono::nanoseconds(over_limit_nanos));
  return_time = over_limit_time;
  EXPECT_NE(pclock.now(), over_limit_nanos);
  EXPECT_LT(std::abs(pclock.now() - over_limit_nanos), 10LL);
}

TEST(PlayerClock, nonzero_start_time)
{
  SteadyTimePoint return_time;
  NowFunction now_fn = [&return_time]() {
      return return_time;
    };
  const ROSTimePoint start_time = 1234567890LL;
  rosbag2_cpp::PlayerClock pclock(start_time, 1.0, now_fn);

  EXPECT_EQ(pclock.now(), start_time);

  return_time = SteadyTimePoint(std::chrono::seconds(1));
  EXPECT_EQ(pclock.now(), start_time + as_nanos(return_time));
}

TEST(PlayerClock, fast_rate)
{
  SteadyTimePoint return_time;
  NowFunction now_fn = [&return_time]() {
      return return_time;
    };
  rosbag2_cpp::PlayerClock pclock(0, 2.5, now_fn);

  const SteadyTimePoint begin_time(std::chrono::seconds(0));
  return_time = begin_time;
  EXPECT_EQ(pclock.now(), as_nanos(begin_time));

  const SteadyTimePoint some_time(std::chrono::seconds(3));
  return_time = some_time;
  EXPECT_EQ(pclock.now(), as_nanos(some_time) * 2.5);
}

TEST(PlayerClock, slow_rate)
{
  SteadyTimePoint return_time;
  NowFunction now_fn = [&return_time]() {
      return return_time;
    };
  rosbag2_cpp::PlayerClock pclock(0, 0.4, now_fn);

  const SteadyTimePoint begin_time(std::chrono::seconds(0));
  return_time = begin_time;
  EXPECT_EQ(pclock.now(), as_nanos(begin_time));

  const SteadyTimePoint some_time(std::chrono::seconds(12));
  return_time = some_time;
  EXPECT_EQ(pclock.now(), as_nanos(some_time) * 0.4);
}
