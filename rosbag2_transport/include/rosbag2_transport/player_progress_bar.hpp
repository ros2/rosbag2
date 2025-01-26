// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_TRANSPORT__PLAYER_PROGRESS_BAR_HPP_
#define ROSBAG2_TRANSPORT__PLAYER_PROGRESS_BAR_HPP_

#include <string>

#include "rclcpp/logger.hpp"
#include "rcutils/time.h"
#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{

class ROSBAG2_TRANSPORT_PUBLIC PlayerProgressBar
{
public:
  enum class PlayerStatus : char
  {
    BURST = 'B',
    DELAYED = 'D',
    PAUSED = 'P',
    RUNNING = 'R',
    STOPPED = 'S',
  };

  explicit PlayerProgressBar(
    rclcpp::Logger logger,
    rcutils_time_point_value_t starting_time,
    rcutils_time_point_value_t ending_time,
    int32_t progress_bar_update_rate = 3,
    int32_t progress_bar_separation_lines = 3);

  virtual ~PlayerProgressBar();

  void print_help_str() const;

  // Update progress bar with an input timestamp,
  // taking into account the update rate set by the user.
  // The function should be called for regular progress bar updates, for example
  // after the recurrent publishing of the messages.
  // Call update_progress_bar_check_rate function only where it cannot run
  // contemporaneously in multiple threads, i.e. function calls are already protected by a mutex.
  // To avoid locking overhead no new mutex inside the function is directly protecting
  // the access to the class attribute progress_bar_last_time_updated_.
  void update_with_limited_rate(
    const rcutils_time_point_value_t & timestamp,
    const PlayerStatus & status);

  // Update progress bar with the current playback timestamp,
  // irrespective of the update rate set by the user.
  // The function should be called for extraordinary progress bar updates, for example
  // when a log message is printed and we want to 'redraw' the progress bar.
  void update(const PlayerStatus & status);

  void draw_progress_bar(
    const rcutils_time_point_value_t & timestamp,
    const PlayerStatus & status);

private:
  rclcpp::Logger logger_;
  double starting_time_secs_ = 0.0;
  double duration_secs_ = 0.0;
  std::string progress_bar_helper_clear_and_move_cursor_down_;
  std::string progress_bar_helper_move_cursor_up_;

  bool enable_progress_bar_;
  bool progress_bar_update_always_;
  rcutils_duration_value_t progress_bar_update_period_;
  std::chrono::steady_clock::time_point progress_bar_last_time_updated_{};
  int32_t progress_bar_separation_lines_ = 3;
  double progress_secs_from_start_ = 0.0;
  double progress_current_time_secs_ = 0.0;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_PROGRESS_BAR_HPP_
