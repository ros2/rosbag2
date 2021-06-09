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

#ifndef ROSBAG2_CPP__CLOCKS__PLAYER_CLOCK_HPP_
#define ROSBAG2_CPP__CLOCKS__PLAYER_CLOCK_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/time.hpp"
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
  class JumpHandler final
  {
public:
    using SharedPtr = std::shared_ptr<JumpHandler>;
    using pre_callback_t = std::function<void ()>;
    using post_callback_t = std::function<void (const rcl_time_jump_t &)>;

    JumpHandler() = delete;
    JumpHandler(const JumpHandler &) = delete;
    const JumpHandler & operator=(const JumpHandler &) = delete;

    /**
     * \brief Creates JumpHandler object with callbacks for jump operation.
     * \param pre_callback Pre-time jump callback. Must be non-throwing.
     * \param post_callback Post-time jump callback. Must be non-throwing.
     * \param threshold Callbacks will be triggered if the time jump is greater then the threshold.
     * \note These callback functions must remain valid as long as the returned shared pointer is
     * valid.
     * \return Shared pointer to the newly created JumpHandler object.
     * \throws std::bad_alloc if the allocation of the JumpHandler fails.
     * \throws std::invalid argument if any of the provided callbacks are nullptr.
     */
    ROSBAG2_CPP_PUBLIC
    static SharedPtr create(
      const pre_callback_t & pre_callback, const post_callback_t & post_callback,
      const rcl_jump_threshold_t & threshold)
    {
      JumpHandler::SharedPtr handler(new JumpHandler(pre_callback, post_callback, threshold));
      if (handler == nullptr) {
        throw std::bad_alloc{};
      }
      return handler;
    }

    /**
     * \brief Equal operator for PlayerClock::JumpHandler class.
     * \param right Right side operand for equal comparison operation.
     * \return true if internal id's of two JumpHandlers match, otherwise false.
     */
    ROSBAG2_CPP_PUBLIC
    bool operator==(const PlayerClock::JumpHandler & right) const
    {
      return id == right.id;
    }

    const pre_callback_t pre_callback;
    const post_callback_t post_callback;
    const rcl_jump_threshold_t notice_threshold;

private:
    uint64_t id;

    JumpHandler(
      const pre_callback_t & pre_callback, const post_callback_t & post_callback,
      const rcl_jump_threshold_t & threshold)
    : pre_callback(pre_callback), post_callback(post_callback), notice_threshold(threshold)
    {
      if (pre_callback == nullptr || post_callback == nullptr) {
        throw std::invalid_argument("Callbacks for JumpHandler shouldn't be nullptr");
      }
      id = create_id();
    }

    static uint64_t create_id()
    {
      static std::atomic<uint64_t> id_count{0};
      return id_count.fetch_add(1, std::memory_order_relaxed) + 1;
    }
  };

  /**
   * Type representing an arbitrary steady time, used to measure real-time durations
   * This type is never exposed by the PlayerClock - it is only used as input to the PlayerClock.
   */
  typedef std::function<std::chrono::steady_clock::time_point()> NowFunction;

  ROSBAG2_CPP_PUBLIC
  virtual ~PlayerClock() = default;

  /**
   * Calculate and return current rcutils_time_point_value_t based on starting time, playback rate, pause state.
   */
  ROSBAG2_CPP_PUBLIC
  virtual rcutils_time_point_value_t now() const = 0;

  /**
   * Try to sleep (non-busy) the current thread until the provided time is reached - according to this Clock
   *
   * Return true if the time has been reached, false if it was not successfully reached after sleeping
   * for the appropriate duration.
   * The user should not take action based on this sleep until it returns true.
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool sleep_until(rcutils_time_point_value_t until) = 0;

  /**
   * \sa sleep_until
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool sleep_until(rclcpp::Time time) = 0;

  /**
   * Change the rate of the flow of time for the clock.
   * \param rate new rate of clock playback
   * \bool false if rate is invalid for the clock implementation
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool set_rate(double rate) = 0;

  /**
   * \return the current playback rate.
   */
  ROSBAG2_CPP_PUBLIC
  virtual double get_rate() const = 0;

  /**
   * Stop the flow of time of the clock.
   * If this changes the pause state, this will wake any waiting `sleep_until`
   */
  ROSBAG2_CPP_PUBLIC
  virtual void pause() = 0;

  /**
   * Start the flow of time of the clock
   * If this changes the pause state, this will wake any waiting `sleep_until`
   */
  ROSBAG2_CPP_PUBLIC
  virtual void resume() = 0;

  /**
   * Return whether the clock is currently paused.
   */
  ROSBAG2_CPP_PUBLIC
  virtual bool is_paused() const = 0;

  /**
   * \brief Change the current ROS time to an arbitrary time.
   * \note This will wake any waiting `sleep_until` and trigger any registered JumpHandler
   * callbacks.
   * \param time_point Time point in ROS playback timeline.
   */
  ROSBAG2_CPP_PUBLIC
  virtual void jump(rcutils_time_point_value_t time_point) = 0;

  /**
   * \sa jump
   */
  ROSBAG2_CPP_PUBLIC
  virtual void jump(rclcpp::Time time) = 0;

  /**
   * \brief Add callbacks to be called when a time jump exceeds a threshold.
   * \details Callbacks specified in JumpHandler object will be called in two cases:
   *   1. use_sim_time is true: if the external time source jumps back in time, or forward
   * farther than the threshold.
   *   2. use_sim_time is false: if jump(time_point) is called and time jumps back or forward
   * farther than the threshold.
   * \param handler Shared pointer to the JumpHandler object returned from JumpHandler::create(..)
   * \throws std::invalid argument if jump threshold has invalid value.
   */
  ROSBAG2_CPP_PUBLIC
  virtual void add_jump_calbacks(PlayerClock::JumpHandler::SharedPtr handler) = 0;

  /**
   * \brief remove jump callbacks from processing list.
   * \param handler Shared pointer to the JumpHandler object returned from JumpHandler::create(..)
   */
  ROSBAG2_CPP_PUBLIC
  virtual void remove_jump_callbacks(PlayerClock::JumpHandler::SharedPtr handler) = 0;
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CLOCKS__PLAYER_CLOCK_HPP_
