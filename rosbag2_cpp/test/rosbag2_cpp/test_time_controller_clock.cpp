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

#include <atomic>
#include <thread>

#include "rosbag2_cpp/clocks/time_controller_clock.hpp"

using namespace testing;  // NOLINT
using SteadyTimePoint = std::chrono::steady_clock::time_point;
using NowFunction = rosbag2_cpp::PlayerClock::NowFunction;

class TimeControllerClockTest : public Test
{
public:
  TimeControllerClockTest()
  {
    now_fn = [this]() {
        return return_time;
      };
  }

  rcutils_time_point_value_t as_nanos(const SteadyTimePoint & time)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
  }

  NowFunction now_fn;
  SteadyTimePoint return_time;  // defaults to 0
  rcutils_time_point_value_t ros_start_time = 0;
};

TEST_F(TimeControllerClockTest, steadytime_precision)
{
  const double playback_rate = 1.0;
  rosbag2_cpp::TimeControllerClock pclock(ros_start_time, playback_rate, now_fn);

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

TEST_F(TimeControllerClockTest, nonzero_start_time)
{
  ros_start_time = 1234567890LL;
  const double playback_rate = 1.0;
  rosbag2_cpp::TimeControllerClock pclock(ros_start_time, playback_rate, now_fn);

  const SteadyTimePoint begin_time(std::chrono::seconds(0));
  return_time = begin_time;
  EXPECT_EQ(pclock.now(), ros_start_time);

  return_time = SteadyTimePoint(std::chrono::seconds(1));
  EXPECT_EQ(pclock.now(), ros_start_time + as_nanos(return_time));
}

TEST_F(TimeControllerClockTest, fast_rate)
{
  const double playback_rate = 2.5;
  rosbag2_cpp::TimeControllerClock pclock(ros_start_time, playback_rate, now_fn);

  const SteadyTimePoint begin_time(std::chrono::seconds(0));
  return_time = begin_time;
  EXPECT_EQ(pclock.now(), as_nanos(begin_time));

  const SteadyTimePoint some_time(std::chrono::seconds(3));
  return_time = some_time;
  EXPECT_EQ(pclock.now(), as_nanos(some_time) * playback_rate);
}

TEST_F(TimeControllerClockTest, slow_rate)
{
  const double playback_rate = 0.4;
  rosbag2_cpp::TimeControllerClock pclock(ros_start_time, playback_rate, now_fn);

  const SteadyTimePoint begin_time(std::chrono::seconds(0));
  return_time = begin_time;
  EXPECT_EQ(pclock.now(), as_nanos(begin_time));

  const SteadyTimePoint some_time(std::chrono::seconds(12));
  return_time = some_time;
  EXPECT_EQ(pclock.now(), as_nanos(some_time) * playback_rate);
}

TEST_F(TimeControllerClockTest, is_paused)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, 1.0);
  EXPECT_FALSE(clock.is_paused());
  clock.pause();
  EXPECT_TRUE(clock.is_paused());
  clock.resume();
  EXPECT_FALSE(clock.is_paused());
}

TEST_F(TimeControllerClockTest, unpaused_sleep_returns_true)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, 1.0);
  clock.resume();
  EXPECT_TRUE(clock.sleep_until(clock.now() + 100));

  clock.pause();
  clock.resume();
  EXPECT_TRUE(clock.sleep_until(clock.now() + RCUTILS_S_TO_NS(1)));
}

TEST_F(TimeControllerClockTest, paused_sleep_returns_false_quickly)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, 1.0);
  clock.pause();
  EXPECT_FALSE(clock.sleep_until(clock.now() + RCUTILS_S_TO_NS(10)));
}


TEST_F(TimeControllerClockTest, paused_now_always_same)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, 1.0);
  clock.pause();
  auto now_start = clock.now();
  EXPECT_EQ(now_start, clock.now());
  clock.sleep_until(clock.now() + RCUTILS_S_TO_NS(1));
  EXPECT_EQ(now_start, clock.now());

  clock.resume();
  EXPECT_NE(now_start, clock.now());
}

TEST_F(TimeControllerClockTest, interrupted_sleep_returns_false_immediately)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, 1.0);
  std::atomic_bool thread_sleep_result{true};
  auto sleep_long_thread = std::thread(
    [&clock, &thread_sleep_result]() {
      bool sleep_result = clock.sleep_until(clock.now() + RCUTILS_S_TO_NS(10));
      thread_sleep_result.store(sleep_result);
    });
  clock.pause();  // Interrupts the long sleep, causing it to return false
  sleep_long_thread.join();
  EXPECT_FALSE(thread_sleep_result);
}
