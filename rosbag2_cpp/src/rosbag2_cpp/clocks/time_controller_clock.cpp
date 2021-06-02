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
#include "rosbag2_cpp/clocks/time_controller_clock.hpp"
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

class TimeControllerClockImpl
{
public:
  /**
   * Stores an exact time match between a system steady clock and the playback ROS clock.
   * This snapshot is taken whenever a factor changes such that a new reference is needed,
   * such as pause, resume, rate change, or jump
   */
  struct TimeReference
  {
    rcutils_time_point_value_t ros;
    std::chrono::steady_clock::time_point steady;
  };

  explicit TimeControllerClockImpl(
    PlayerClock::NowFunction now_fn, std::chrono::milliseconds sleep_time_while_paused)
  : now_fn(now_fn),
    sleep_time_while_paused(sleep_time_while_paused)
  {}
  virtual ~TimeControllerClockImpl() = default;

  /// Return the total nanoseconds of an arbitrary duration type.
  template<typename T>
  rcutils_duration_value_t duration_nanos(const T & duration) const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  }

  /// Convert an arbitrary SteadyTime to a ROSTime, based on the current reference snapshot.
  rcutils_time_point_value_t steady_to_ros(std::chrono::steady_clock::time_point steady_time) const
  RCPPUTILS_TSA_REQUIRES(state_mutex)
  {
    return reference.ros + static_cast<rcutils_duration_value_t>(
      rate * duration_nanos(steady_time - reference.steady));
  }

  /// Convert an arbitrary ROSTime to a SteadyTime, based on the current reference snapshot.
  std::chrono::steady_clock::time_point ros_to_steady(rcutils_time_point_value_t ros_time) const
  RCPPUTILS_TSA_REQUIRES(state_mutex)
  {
    const auto diff_nanos = static_cast<rcutils_duration_value_t>(
      (ros_time - reference.ros) / rate);
    return reference.steady + std::chrono::nanoseconds(diff_nanos);
  }

  /// Return the current ROS time right now, based on current settings.
  rcutils_time_point_value_t ros_now() const
  RCPPUTILS_TSA_REQUIRES(state_mutex)
  {
    if (paused) {
      return reference.ros;
    }
    return steady_to_ros(now_fn());
  }

  /// Take a new reference snapshot, matching `ros_time` to the current steady time
  void snapshot(rcutils_time_point_value_t ros_time)
  RCPPUTILS_TSA_REQUIRES(state_mutex)
  {
    reference.ros = ros_time;
    reference.steady = now_fn();
  }

  /**
   * Take a new reference snaphot to match the current ROStime to the current SteadyTime
   * This is needed when changing a setting such as pause or rate.
   */
  void snapshot()
  RCPPUTILS_TSA_REQUIRES(state_mutex)
  {
    snapshot(ros_now());
  }

  /// \brief Adjust internal clock to the specified timestamp
  /// \details It will change the current internally maintained offset so that next published time
  /// is different.
  /// \note Will trigger any registered JumpHandler callbacks.
  /// \param time_point Time point in ROS playback timeline.
  void jump(rcutils_time_point_value_t ros_time)
  {
    rcl_duration_t time_jump_delta;
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      time_jump_delta.nanoseconds = ros_time - steady_to_ros(now_fn());
    }

    rcl_time_jump_t time_jump{};
    time_jump.clock_change = RCL_ROS_TIME_NO_CHANGE;
    time_jump.delta = time_jump_delta;

    process_callbacks_before_jump(time_jump);
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      snapshot(ros_time);
    }
    process_callbacks_after_jump(time_jump);
    cv.notify_all();
  }

  /// \brief Add callbacks to be called when a time jump exceeds a threshold.
  /// \param handler Shared pointer to the JumpHandler object returned from JumpHandler::create(..)
  /// \throws std::invalid argument if jump threshold has invalid value.
  void add_jump_callbacks(PlayerClock::JumpHandler::SharedPtr handler)
  {
    if (handler->notice_threshold.min_forward.nanoseconds < 0) {
      throw std::invalid_argument("forward jump threshold must be positive or zero");
    }
    if (handler->notice_threshold.min_backward.nanoseconds > 0) {
      throw std::invalid_argument("backward jump threshold must be negative or zero");
    }

    std::lock_guard<std::mutex> lock(callback_list_mutex_);
    for (auto const & registered_handler : callback_list_) {
      if (*registered_handler == *handler) {
        return;  // Already have this callback in the list.
      }
    }
    callback_list_.push_back(handler);
  }

  /// \brief remove jump callbacks from processing list.
  /// \param handler Shared pointer to the JumpHandler object returned from JumpHandler::create(..)
  void remove_jump_callbacks(PlayerClock::JumpHandler::SharedPtr handler)
  {
    std::lock_guard<std::mutex> lock(callback_list_mutex_);
    for (auto it = callback_list_.begin(); it != callback_list_.end(); ++it) {
      if (**it == *handler) {
        callback_list_.erase(it);
        return;
      }
    }
  }

  const PlayerClock::NowFunction now_fn;
  const std::chrono::milliseconds sleep_time_while_paused;

  std::mutex state_mutex;
  std::condition_variable cv RCPPUTILS_TSA_GUARDED_BY(state_mutex);
  double rate RCPPUTILS_TSA_GUARDED_BY(state_mutex) = 1.0;
  bool paused RCPPUTILS_TSA_GUARDED_BY(state_mutex) = false;
  TimeReference reference RCPPUTILS_TSA_GUARDED_BY(state_mutex);

private:
  std::mutex callback_list_mutex_;
  std::vector<PlayerClock::JumpHandler::SharedPtr> callback_list_
    RCPPUTILS_TSA_GUARDED_BY(callback_list_mutex_);

  void process_callbacks_before_jump(const rcl_time_jump_t & time_jump)
  {
    std::lock_guard<std::mutex> lock(callback_list_mutex_);
    for (auto const & handler : callback_list_) {
      process_callback(handler, time_jump, true);
    }
  }

  void process_callbacks_after_jump(const rcl_time_jump_t & time_jump)
  {
    std::lock_guard<std::mutex> lock(callback_list_mutex_);
    for (auto const & handler : callback_list_) {
      process_callback(handler, time_jump, false);
    }
  }

  static void process_callback(
    PlayerClock::JumpHandler::SharedPtr handler, const rcl_time_jump_t & time_jump,
    bool before_jump) RCPPUTILS_TSA_REQUIRES(callback_list_mutex_)
  {
    bool is_clock_change = time_jump.clock_change == RCL_ROS_TIME_ACTIVATED ||
      time_jump.clock_change == RCL_ROS_TIME_DEACTIVATED;
    if ((is_clock_change && handler->notice_threshold.on_clock_change) ||
      (handler->notice_threshold.min_backward.nanoseconds != 0 &&
      time_jump.delta.nanoseconds < 0 &&
      time_jump.delta.nanoseconds <= handler->notice_threshold.min_backward.nanoseconds) ||
      (handler->notice_threshold.min_forward.nanoseconds != 0 &&
      time_jump.delta.nanoseconds > 0 &&
      time_jump.delta.nanoseconds >= handler->notice_threshold.min_forward.nanoseconds))
    {
      if (before_jump && handler->pre_callback) {
        handler->pre_callback();
      } else if (!before_jump && handler->post_callback) {
        handler->post_callback(time_jump);
      }
    }
  }
};

TimeControllerClock::TimeControllerClock(
  rcutils_time_point_value_t starting_time,
  NowFunction now_fn,
  std::chrono::milliseconds sleep_time_while_paused)
: impl_(std::make_unique<TimeControllerClockImpl>(now_fn, sleep_time_while_paused))
{
  if (now_fn == nullptr) {
    throw std::invalid_argument("TimeControllerClock now_fn must be non-empty.");
  }
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  impl_->reference.ros = starting_time;
  impl_->reference.steady = impl_->now_fn();
}

TimeControllerClock::~TimeControllerClock()
{}

rcutils_time_point_value_t TimeControllerClock::now() const
{
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  return impl_->ros_now();
}

bool TimeControllerClock::sleep_until(rcutils_time_point_value_t until)
{
  {
    TSAUniqueLock lock(impl_->state_mutex);
    if (impl_->paused) {
      impl_->cv.wait_for(lock, impl_->sleep_time_while_paused);
    } else {
      const auto steady_until = impl_->ros_to_steady(until);
      impl_->cv.wait_until(lock, steady_until);
    }
    if (impl_->paused) {
      // Don't allow publishing any messages while paused
      // even if the time was technically reached by the time of wakeup
      return false;
    }
  }
  return now() >= until;
}

bool TimeControllerClock::sleep_until(rclcpp::Time until)
{
  return sleep_until(until.nanoseconds());
}

bool TimeControllerClock::set_rate(double rate)
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

void TimeControllerClock::add_jump_calbacks(PlayerClock::JumpHandler::SharedPtr handler)
{
  impl_->add_jump_callbacks(handler);
}

void TimeControllerClock::remove_jump_callbacks(PlayerClock::JumpHandler::SharedPtr handler)
{
  impl_->remove_jump_callbacks(handler);
}

double TimeControllerClock::get_rate() const
{
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  return impl_->rate;
}

void TimeControllerClock::pause()
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

void TimeControllerClock::resume()
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

bool TimeControllerClock::is_paused() const
{
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  return impl_->paused;
}

void TimeControllerClock::jump(rcutils_time_point_value_t ros_time)
{
  impl_->jump(ros_time);
}

void TimeControllerClock::jump(rclcpp::Time ros_time)
{
  jump(ros_time.nanoseconds());
}

}  // namespace rosbag2_cpp
