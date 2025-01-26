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

#ifndef ROSBAG2_TRANSPORT__PLAYER_PROGRESS_BAR_IMPL_HPP_
#define ROSBAG2_TRANSPORT__PLAYER_PROGRESS_BAR_IMPL_HPP_

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <iostream>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/time.h"
#include "rosbag2_transport/player_progress_bar.hpp"

namespace rosbag2_transport
{
class PlayerProgressBarImpl {
public:
  using PlayerStatus = PlayerProgressBar::PlayerStatus;

  PlayerProgressBarImpl(
    rclcpp::Logger logger,
    rcutils_time_point_value_t starting_time,
    rcutils_time_point_value_t ending_time,
    int32_t progress_bar_update_rate,
    int32_t progress_bar_separation_lines)
  : logger_(std::move(logger)),
    enable_progress_bar_(progress_bar_update_rate != 0),
    progress_bar_update_always_(progress_bar_update_rate < 0),
    progress_bar_update_period_(progress_bar_update_rate != 0 ?
      RCUTILS_S_TO_NS(1.0 / progress_bar_update_rate) :
      std::numeric_limits<rcutils_duration_value_t>::max()),
    progress_bar_separation_lines_(progress_bar_separation_lines)
  {
    starting_time_secs_ = RCUTILS_NS_TO_S(
      static_cast<double>(std::min(starting_time, ending_time)));
    duration_secs_ = RCUTILS_NS_TO_S(
      std::max(static_cast<double>(ending_time - starting_time), 0.0));
    progress_secs_from_start_ = starting_time_secs_;
    progress_current_time_secs_ = starting_time_secs_;
    std::ostringstream oss_clear_and_move_cursor_down;
    for (int i = 0; i < progress_bar_separation_lines_; i++) {
      oss_clear_and_move_cursor_down << "\033[2K\n";
    }
    oss_clear_and_move_cursor_down << "\033[2K";
    progress_bar_helper_clear_and_move_cursor_down_ = oss_clear_and_move_cursor_down.str();
    std::ostringstream oss_move_cursor_up;
    oss_move_cursor_up << "\033[" << progress_bar_separation_lines_ + 1 << "F";
    progress_bar_helper_move_cursor_up_ = oss_move_cursor_up.str();
  }

  ~PlayerProgressBarImpl()
  {
    // arrange cursor position to be after the progress bar
    if (enable_progress_bar_) {
      std::ostringstream oss;
      oss << "\033[" << progress_bar_separation_lines_ + 1 << "B\n";
      std::cout << oss.str() << std::flush;
    }
  }

  void print_help_str() const
  {
    std::string help_str;
    if (enable_progress_bar_) {
      if (progress_bar_update_always_) {
        help_str = "Progress bar enabled for every message.";
      } else {
        std::ostringstream oss;
        oss << "Progress bar enabled at " <<
          (1.0 / (progress_bar_update_period_ / (1000LL * 1000LL))) / 1000LL << " Hz";
        help_str = oss.str();
      }
      RCLCPP_INFO_STREAM(logger_, help_str);
      std::string help_str2 =
        "Progress bar [?]: [R]unning, [P]aused, [B]urst, [D]elayed, [S]topped";
      RCLCPP_INFO_STREAM(logger_, help_str2);
    } else {
      help_str = "Progress bar disabled";
      RCLCPP_INFO_STREAM(logger_, help_str);
    }
  }

  void update_with_limited_rate(
    const rcutils_time_point_value_t & timestamp,
    const PlayerStatus & status)
  {
    if (!enable_progress_bar_) {
      return;
    }

    // If we are not updating the progress bar for every call, check if we should update it now
    // based on the update rate set by the user
    if (!progress_bar_update_always_) {
      std::chrono::steady_clock::time_point steady_time_now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::nanoseconds>(
        steady_time_now - progress_bar_last_time_updated_).count() < progress_bar_update_period_)
      {
        return;
      }
      progress_bar_last_time_updated_ = steady_time_now;
    }

    draw_progress_bar(timestamp, status);
  }

  void update(const PlayerStatus & status)
  {
    if (!enable_progress_bar_) {
      return;
    }
    // Update progress bar irrespective of the update rate set by the user.
    draw_progress_bar(-1, status);
  }

  void draw_progress_bar(
    const rcutils_time_point_value_t & timestamp,
    const PlayerStatus & status)
  {
    if (timestamp > 0) {
      progress_current_time_secs_ = RCUTILS_NS_TO_S(static_cast<double>(timestamp));
      progress_secs_from_start_ = progress_current_time_secs_ - starting_time_secs_;
    }
    std::ostringstream oss;
    oss <<
        // Clear and print newlines
      progress_bar_helper_clear_and_move_cursor_down_ <<
        // Print progress bar
      "====== Playback Progress ======\n" <<
      "[" << std::setw(13) << std::fixed << std::setprecision(9) << progress_current_time_secs_ <<
      "] Duration " << std::setprecision(2) << progress_secs_from_start_ <<
        // Spaces at the end are used to clear any previous progress bar in case the new one is
        // shorter, which can happen when the playback starts a new loop.
      "/" << duration_secs_ << " [" << static_cast<char>(status) << "]      " <<
        // Go up to the beginning of the blank lines
      progress_bar_helper_move_cursor_up_;
    std::cout << oss.str() << std::flush;
  }

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

#endif  // ROSBAG2_TRANSPORT__PLAYER_PROGRESS_BAR_IMPL_HPP_
