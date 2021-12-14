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

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rcpputils/thread_safety_annotations.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/player_clock.hpp"
#include "rosbag2_cpp/types.hpp"

namespace
{
/**
 * Trivial std::unique_lock wrapper providing constructor that allows Clang Thread Safety Analysis.
 * The std::unique_lock does not have these annotations.
 */
class RCPPUTILS_TSA_SCOPED_CAPABILITY TSAUniqueLock : public std::unique_lock<std::mutex>
{
public:
  explicit TSAUniqueLock(std::mutex & mu) RCPPUTILS_TSA_ACQUIRE(mu)
  : std::unique_lock<std::mutex>(mu)
  {}

  ~TSAUniqueLock() RCPPUTILS_TSA_RELEASE() {}
};
}  // namespace

namespace rosbag2_cpp
{

class PlayerClockImpl
{
public:
  /**
   * Stores an exact time match between a source-time and the bag-time for playback.
   * This snapshot is taken whenever a factor changes such that a new reference is needed,
   * such as pause, resume, rate change, or offset override
   */
  struct TimeReference
  {
    rcutils_time_point_value_t source;
    rcutils_time_point_value_t bag;
  };

  explicit PlayerClockImpl(
    rclcpp::Clock::SharedPtr clock,
    std::chrono::milliseconds sleep_time_while_paused,
    bool start_paused)
  :
    clock(clock),
    sleep_time_while_paused(sleep_time_while_paused),
    paused(start_paused)
  {}
  virtual ~PlayerClockImpl() = default;

  /// Convert an arbitrary source-time to bag-time, based on the current reference snapshot.
  rcutils_time_point_value_t source_to_bag(rcutils_time_point_value_t source_time) const
  RCPPUTILS_TSA_REQUIRES(state_mutex)
  {
    return reference.bag + (rate * (source_time - reference.source));
  }

  /// Convert an arbitrary bag-time to a source-time, based on the current reference snapshot.
  rcutils_time_point_value_t bag_to_source(rcutils_time_point_value_t bag_time) const
  RCPPUTILS_TSA_REQUIRES(state_mutex)
  {
    const auto diff_nanos = (bag_time - reference.bag) / rate;
    return reference.source + diff_nanos;
  }

  /// Return the current bag time right now, based on current settings.
  rcutils_time_point_value_t bag_now() const
  RCPPUTILS_TSA_REQUIRES(state_mutex)
  {
    if (paused) {
      return reference.bag;
    }
    return source_to_bag(clock->now().nanoseconds());
  }

  /// Take a new reference snapshot, matching bag-time to the current source-time
  TimeReference snapshot(rcutils_time_point_value_t bag_time) const
  RCPPUTILS_TSA_REQUIRES(state_mutex)
  {
    TimeReference ref;
    ref.source = clock->now().nanoseconds();
    ref.bag = bag_time;
    return ref;
  }

  /// Take a new reference snaphot to match the current bag-time to the current source-time
  /// This is needed when changing a setting such as pause or rate.
  TimeReference snapshot() const
  RCPPUTILS_TSA_REQUIRES(state_mutex)
  {
    return snapshot(bag_now());
  }


  /// \brief Adjust internal clock to the specified timestamp.
  /// \details It will change the internal offset so that next returned time is different.
  /// \note Will trigger any registered JumpHandler callbacks.
  /// \param bag_time Time point in bag playback timeline.
  void override_offset(rcutils_time_point_value_t bag_time)
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    reference = snapshot(bag_time);
    cv.notify_all();
  }

  const PlayerClock::NowFunction now_fn;
  rclcpp::Clock::SharedPtr clock;
  rclcpp::JumpHandler::SharedPtr jump_handler;
  const std::chrono::milliseconds sleep_time_while_paused;

  std::mutex state_mutex;
  std::condition_variable cv RCPPUTILS_TSA_GUARDED_BY(state_mutex);
  double rate RCPPUTILS_TSA_GUARDED_BY(state_mutex) = 1.0;
  bool paused RCPPUTILS_TSA_GUARDED_BY(state_mutex) = false;
  TimeReference reference RCPPUTILS_TSA_GUARDED_BY(state_mutex);
};

PlayerClock::PlayerClock(
  rcutils_time_point_value_t starting_time,
  rclcpp::Clock::SharedPtr clock,
  std::chrono::milliseconds sleep_time_while_paused,
  bool paused)
: impl_(std::make_unique<PlayerClockImpl>(clock, sleep_time_while_paused, paused))
{
  if (clock == nullptr) {
    throw std::invalid_argument("PlayerClock's ROS clock must be non-null.");
  }
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  impl_->snapshot(starting_time);
  if (clock->get_clock_type() == RCL_ROS_TIME) {
    rcl_jump_threshold_t threshold;
    threshold.on_clock_change = true;
    // 0 is disable, so -1 and 1 are smallest possible time changes
    threshold.min_backward.nanoseconds = -1;
    threshold.min_forward.nanoseconds = 1;
    impl_->jump_handler = clock->create_jump_callback(
      nullptr,
      [this](const rcl_time_jump_t & jump) {
        if (jump.clock_change != RCL_ROS_TIME_NO_CHANGE) {
        //   time_source_changed = true;
        }
        impl_->cv.notify_one();
      }, threshold);
  }
}

PlayerClock::~PlayerClock()
{}

rcutils_time_point_value_t PlayerClock::now() const
{
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  return impl_->bag_now();
}

bool PlayerClock::sleep_until(rcutils_time_point_value_t until)
{
  {
    TSAUniqueLock lock(impl_->state_mutex);
    if (impl_->paused) {
      impl_->cv.wait_for(lock, impl_->sleep_time_while_paused);
    } else if (impl_->clock->get_clock_type() == RCL_STEADY_TIME) {
      const auto rcl_entry = impl_->clock->now();
      const auto chrono_entry = std::chrono::steady_clock::now();
      const rcutils_duration_value_t delta_t = until - rcl_entry.nanoseconds();
      const auto chrono_until =
        chrono_entry + std::chrono::nanoseconds(delta_t);

      impl_->cv.wait_until(lock, chrono_until);
    } else if (impl_->clock->get_clock_type() == RCL_ROS_TIME) {
      if (!impl_->clock->ros_time_is_active()) {
        throw std::runtime_error("PlayerClock has RCL_ROS_TIME but ros time is not active! "
          "Invalid configuration.");
      }
      // TODO!
      while (now() < until && rclcpp::ok()) {
        impl_->cv.wait_for(lock, std::chrono::milliseconds(10));
      }
    } else {
      throw std::runtime_error("PlayerClock's ROS clock has unsupported type. Shouldn't happen.");
    }
    if (impl_->paused) {
      // Don't allow publishing any messages while paused
      // even if the time was technically reached by the time of wakeup
      return false;
    }
  }
  return now() >= until;
}

bool PlayerClock::set_rate(double rate)
{
  if (rate <= 0) {
    return false;
  }
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  if (impl_->rate == rate) {
    return true;
  }
  impl_->snapshot();
  impl_->rate = rate;
  impl_->cv.notify_all();
  return true;
}

double PlayerClock::get_rate() const
{
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  return impl_->rate;
}

void PlayerClock::pause()
{
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  if (impl_->paused) {
    return;
  }
  // Take snapshot before changing state
  impl_->snapshot();
  impl_->paused = true;
  impl_->cv.notify_all();
}

void PlayerClock::resume()
{
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  if (!impl_->paused) {
    return;
  }
  // Take snapshot before changing state
  impl_->snapshot();
  impl_->paused = false;
  impl_->cv.notify_all();
}

bool PlayerClock::is_paused() const
{
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  return impl_->paused;
}

void PlayerClock::override_offset(rcutils_time_point_value_t bag_time)
{
  impl_->override_offset(bag_time);
}

}  // namespace rosbag2_cpp
