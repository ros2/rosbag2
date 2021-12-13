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
using SteadyTimeDurationSecT = std::chrono::duration<int>;
using NowFunction = rosbag2_cpp::PlayerClock::NowFunction;

class TimeControllerClockTest : public Test
{
public:
  TimeControllerClockTest()
  : return_time(std::chrono::nanoseconds(0))
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

TEST_F(TimeControllerClockTest, must_provide_now_fn)
{
  NowFunction empty_now;
  EXPECT_THROW(
    rosbag2_cpp::TimeControllerClock(ros_start_time, empty_now),
    std::invalid_argument);
}

TEST_F(TimeControllerClockTest, steadytime_precision)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, now_fn);

  const SteadyTimePoint begin_time(std::chrono::seconds(0));
  return_time = begin_time;
  EXPECT_EQ(clock.now(), as_nanos(begin_time));

  const SteadyTimePoint ten_seconds(std::chrono::seconds(10));
  return_time = ten_seconds;
  EXPECT_EQ(clock.now(), as_nanos(ten_seconds));

  // NOTE: this would have already lost precision at 100 seconds if we were multiplying by float
  const SteadyTimePoint hundred_seconds(std::chrono::seconds(100));
  return_time = hundred_seconds;
  EXPECT_EQ(clock.now(), as_nanos(hundred_seconds));

  const int64_t near_limit_nanos = 1LL << 61;
  const auto near_limit_time = SteadyTimePoint(std::chrono::nanoseconds(near_limit_nanos));
  return_time = near_limit_time;
  EXPECT_EQ(clock.now(), near_limit_nanos);

  // Expect to lose exact equality at this point due to precision limit of double*int mult
  const int64_t over_limit_nanos = (1LL << 61) + 1LL;
  const auto over_limit_time = SteadyTimePoint(std::chrono::nanoseconds(over_limit_nanos));
  return_time = over_limit_time;
  EXPECT_NE(clock.now(), over_limit_nanos);
  EXPECT_LT(std::abs(clock.now() - over_limit_nanos), 10LL);
}

TEST_F(TimeControllerClockTest, nonzero_start_time)
{
  ros_start_time = 1234567890LL;
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, now_fn);
  clock.set_rate(1.0);

  const SteadyTimePoint begin_time(std::chrono::seconds(0));
  return_time = begin_time;
  EXPECT_EQ(clock.now(), ros_start_time);

  return_time = SteadyTimePoint(std::chrono::seconds(1));
  EXPECT_EQ(clock.now(), ros_start_time + as_nanos(return_time));
}

TEST_F(TimeControllerClockTest, fast_rate)
{
  const double playback_rate = 2.5;
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, now_fn);
  clock.set_rate(playback_rate);

  const SteadyTimePoint begin_time(std::chrono::seconds(0));
  return_time = begin_time;
  EXPECT_EQ(clock.now(), as_nanos(begin_time));

  const SteadyTimePoint some_time(std::chrono::seconds(3));
  return_time = some_time;
  EXPECT_EQ(clock.now(), as_nanos(some_time) * playback_rate);
}

TEST_F(TimeControllerClockTest, slow_rate)
{
  const double playback_rate = 0.4;
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, now_fn);
  clock.set_rate(playback_rate);

  const SteadyTimePoint begin_time(std::chrono::seconds(0));
  return_time = begin_time;
  EXPECT_EQ(clock.now(), as_nanos(begin_time));

  const SteadyTimePoint some_time(std::chrono::seconds(12));
  return_time = some_time;
  EXPECT_EQ(clock.now(), as_nanos(some_time) * playback_rate);
}

TEST_F(TimeControllerClockTest, invalid_rate)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, now_fn);
  EXPECT_FALSE(clock.set_rate(-5));
  EXPECT_FALSE(clock.set_rate(0));
}

TEST_F(TimeControllerClockTest, is_paused)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time);
  EXPECT_FALSE(clock.is_paused());
  clock.pause();
  EXPECT_TRUE(clock.is_paused());
  clock.resume();
  EXPECT_FALSE(clock.is_paused());
}

TEST_F(TimeControllerClockTest, unpaused_sleep_returns_true)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time);
  clock.resume();
  EXPECT_TRUE(clock.sleep_until(clock.now() + 100));

  clock.pause();
  clock.resume();
  EXPECT_TRUE(clock.sleep_until(clock.now() + RCUTILS_S_TO_NS(1)));
}

TEST_F(TimeControllerClockTest, paused_sleep_returns_false_quickly)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time);
  clock.pause();
  EXPECT_FALSE(clock.sleep_until(clock.now() + RCUTILS_S_TO_NS(10)));
}

TEST_F(TimeControllerClockTest, paused_now_always_same)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time);
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
  rosbag2_cpp::TimeControllerClock clock(ros_start_time);
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

TEST_F(TimeControllerClockTest, resumes_at_correct_ros_time)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, now_fn);

  // Time passes while paused, no ROS time passes
  clock.pause();
  // reference at pause is R(0) / S(0)
  return_time = SteadyTimePoint(std::chrono::seconds(1));
  clock.resume();
  EXPECT_EQ(clock.now(), RCUTILS_S_TO_NS(0));

  // time passes while running, equal ROS time should pass
  return_time = SteadyTimePoint(std::chrono::seconds(2));
  EXPECT_EQ(clock.now(), RCUTILS_S_TO_NS(1));

  // Lots of time passes while paused, still no ROS time passes
  clock.pause();
  return_time = SteadyTimePoint(std::chrono::seconds(10));
  clock.resume();
  EXPECT_EQ(clock.now(), RCUTILS_S_TO_NS(1));
}

TEST_F(TimeControllerClockTest, change_rate)
{
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, now_fn);
  rcutils_time_point_value_t expected_ros_time = 0;

  // 1 second passes for both
  return_time += std::chrono::seconds(1);
  expected_ros_time += RCUTILS_S_TO_NS(1);
  EXPECT_EQ(clock.now(), expected_ros_time);

  // 1 steady second passes, 2 ros seconds
  clock.set_rate(2.0);
  return_time += std::chrono::seconds(1);
  expected_ros_time += RCUTILS_S_TO_NS(2);
  EXPECT_EQ(clock.now(), expected_ros_time);

  clock.pause();
  clock.set_rate(0.5);
  return_time += std::chrono::seconds(1);
  // ROS time doesn't proceed while paused
  EXPECT_EQ(clock.now(), expected_ros_time);

  clock.resume();
  return_time += std::chrono::seconds(2);
  expected_ros_time += RCUTILS_S_TO_NS(1);
  EXPECT_EQ(clock.now(), expected_ros_time);
}

TEST_F(TimeControllerClockTest, jump)
{
  rcutils_time_point_value_t expected_ros_time = RCUTILS_S_TO_NS(10);
  rosbag2_cpp::TimeControllerClock clock(ros_start_time, now_fn);

  clock.override_offset(expected_ros_time);
  EXPECT_EQ(clock.now(), expected_ros_time);

  return_time += std::chrono::seconds(10);
  expected_ros_time += RCUTILS_S_TO_NS(10);
  EXPECT_EQ(clock.now(), expected_ros_time);

  clock.override_offset(rclcpp::Time(100, 0));
  return_time += std::chrono::seconds(1);
  EXPECT_EQ(clock.now(), RCUTILS_S_TO_NS(101));

  // Jump backward
  clock.override_offset(rclcpp::Time(50, 0));
  return_time += std::chrono::seconds(2);
  EXPECT_EQ(clock.now(), RCUTILS_S_TO_NS(52));
}

TEST_F(TimeControllerClockTest, jump_forward_with_playback_rate)
{
  // Use non zero start time along side with payback rate differ than 1.0
  ros_start_time = RCUTILS_S_TO_NS(1);
  const SteadyTimePoint wall_time_begin(std::chrono::seconds(2));
  return_time = wall_time_begin;  // Init now_fn() to some non zero wall_time_begin
  const double playback_rate = 1.5;
  rosbag2_cpp::TimeControllerClock testing_clock(ros_start_time, now_fn);
  testing_clock.set_rate(playback_rate);

  const rcutils_time_point_value_t ros_time_point_for_jump = ros_start_time + RCUTILS_S_TO_NS(3);
  testing_clock.override_offset(ros_time_point_for_jump);

  // Emulate wall clock ticks
  const SteadyTimePoint current_wall_time = wall_time_begin + SteadyTimeDurationSecT(5);
  return_time = current_wall_time;  // now_fn() will return wall_time_begin + 5 sec

  // Jump shouldn't take in to account playback rate when jumping in time,
  // although TimeControllerClock::now() should respect playback rate after jump.
  rcutils_time_point_value_t expected_ros_time = ros_time_point_for_jump +
    static_cast<rcutils_time_point_value_t>(playback_rate * (as_nanos(current_wall_time) -
    as_nanos(wall_time_begin)));

  EXPECT_EQ(testing_clock.now(), expected_ros_time);
}

TEST_F(TimeControllerClockTest, jump_backward_with_playback_rate)
{
  // Use non zero start time along side with payback rate differ than 1.0
  ros_start_time = RCUTILS_S_TO_NS(5);
  const SteadyTimePoint wall_time_begin(std::chrono::seconds(2));
  return_time = wall_time_begin;  // Init now_fn() to some non zero wall_time_begin
  const double playback_rate = 1.5;
  rosbag2_cpp::TimeControllerClock testing_clock(ros_start_time, now_fn);
  testing_clock.set_rate(playback_rate);

  const rcutils_time_point_value_t ros_time_point_for_jump = ros_start_time - RCUTILS_S_TO_NS(3);
  testing_clock.override_offset(ros_time_point_for_jump);

  // Emulate wall clock ticks
  const SteadyTimePoint current_wall_time = wall_time_begin + SteadyTimeDurationSecT(5);
  return_time = current_wall_time;  // now_fn() will return wall_time_begin + 5 sec

  // Jump shouldn't take in to account playback rate when jumping in time,
  // although TimeControllerClock::now() should respect playback rate after jump.
  rcutils_time_point_value_t expected_ros_time = ros_time_point_for_jump +
    static_cast<rcutils_time_point_value_t>(playback_rate * (as_nanos(current_wall_time) -
    as_nanos(wall_time_begin)));

  EXPECT_EQ(testing_clock.now(), expected_ros_time);
}
