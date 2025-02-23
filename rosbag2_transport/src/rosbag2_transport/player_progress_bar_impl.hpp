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
#include <atomic>
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
    std::ostream & output_stream,
    rcutils_time_point_value_t starting_time,
    rcutils_time_point_value_t ending_time,
    int32_t progress_bar_update_rate,
    uint32_t progress_bar_separation_lines)
  : o_stream_(output_stream),
    enable_progress_bar_(progress_bar_update_rate != 0),
    progress_bar_update_period_ms_(progress_bar_update_rate > 0 ?
      (1000 / progress_bar_update_rate) : 0),
    progress_bar_separation_lines_(progress_bar_separation_lines)
  {
    starting_time_secs_ = RCUTILS_NS_TO_S(
      static_cast<double>(std::min(starting_time, ending_time)));
    duration_secs_ = RCUTILS_NS_TO_S(
      std::max(static_cast<double>(ending_time - starting_time), 0.0));
    progress_current_time_secs_ = starting_time_secs_;
    progress_secs_from_start_ = progress_current_time_secs_ - starting_time_secs_;
    std::ostringstream oss_clear_and_move_cursor_down;

    for (uint32_t i = 0; i < progress_bar_separation_lines_; i++) {
      // The ANSI control code "\033[2K" is used to clear an entire line in the terminal.
      // Cleanup current line and jump down to one new line with "\n"
      oss_clear_and_move_cursor_down << "\033[2K\n";
    }

    progress_bar_helper_move_cursor_down_ = oss_clear_and_move_cursor_down.str();
    std::ostringstream oss_move_cursor_up;
    // Move cursor up by a specific number of lines using "Cursor Up" '\033[<N>A' ANSI control code
    oss_move_cursor_up << "\033[" <<
      progress_bar_separation_lines_ + progress_bar_lines_count_ << "A";
    progress_bar_helper_move_cursor_up_ = oss_move_cursor_up.str();
  }

  ~PlayerProgressBarImpl()
  {
    // arrange cursor position to be after the progress bar
    if (enable_progress_bar_) {
      std::stringstream ss;
      ss << "\033[" << progress_bar_separation_lines_ + progress_bar_lines_count_ << "B";
      o_stream_ << ss.rdbuf() << std::flush;
    }
  }

  void print_help_str() const
  {
    std::stringstream ss;
    if (enable_progress_bar_) {
      if (progress_bar_update_period_ms_ > 0) {
        ss << "Progress bar enabled at " << (1000 / progress_bar_update_period_ms_) << " Hz.\n";
      } else {
        ss << "Progress bar enabled for every message.\n";
      }
      ss << "Progress bar [?]: [R]unning, [P]aused, [B]urst, [D]elayed, [S]topped\n";
    }
    o_stream_ << ss.rdbuf() << std::flush;
  }

  void update_with_limited_rate(
    const PlayerStatus & status,
    const rcutils_time_point_value_t & timestamp)
  {
    current_player_status_.store(status);
    if (!enable_progress_bar_) {
      return;
    }

    // If we are not updating the progress bar for every call, check if we should update it now
    // based on the update rate set by the user
    if (progress_bar_update_period_ms_ > 0) {
      std::chrono::steady_clock::time_point steady_time_now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(
        steady_time_now - progress_bar_last_time_updated_).count() < progress_bar_update_period_ms_)
      {
        return;
      }
      progress_bar_last_time_updated_ = steady_time_now;
    }

    draw_progress_bar(status, timestamp);
  }

  void update(const PlayerStatus & status, const rcutils_time_point_value_t & timestamp = -1)
  {
    current_player_status_.store(status);
    if (!enable_progress_bar_) {
      return;
    }
    // Update progress bar irrespective of the update rate set by the user.
    draw_progress_bar(status, timestamp);
  }

  void draw_progress_bar(
    const PlayerStatus & status,
    const rcutils_time_point_value_t & timestamp = -1)
  {
    if (timestamp >= 0) {
      progress_current_time_secs_ = RCUTILS_NS_TO_S(static_cast<double>(timestamp));
      progress_secs_from_start_ = progress_current_time_secs_ - starting_time_secs_;
    }
    std::stringstream ss;
    ss <<
        // Clear and print newlines
      progress_bar_helper_move_cursor_down_ <<
      // Print progress bar
      // Use "\033[2K" ANSI control code to clear an entire line in the terminal before output.
      "\033[2K====== Playback Progress ======\n\033[2K" <<
      "[" << std::fixed << std::setprecision(9) << progress_current_time_secs_ <<
      "] Duration " << std::setprecision(2) << progress_secs_from_start_ <<
      "/" << duration_secs_ << " [" << static_cast<char>(status) << "]\n" <<
        // Go up to the beginning of the blank lines
      progress_bar_helper_move_cursor_up_;
    o_stream_ << ss.rdbuf() << std::flush;
  }

  std::atomic<PlayerStatus> current_player_status_{PlayerStatus::STOPPED};

private:
  std::ostream & o_stream_;
  double starting_time_secs_ = 0.0;
  double duration_secs_ = 0.0;
  std::string progress_bar_helper_move_cursor_down_;
  std::string progress_bar_helper_move_cursor_up_;
  bool enable_progress_bar_;
  uint16_t progress_bar_update_period_ms_;
  std::chrono::steady_clock::time_point progress_bar_last_time_updated_{};
  uint32_t progress_bar_separation_lines_ = 3;
  double progress_secs_from_start_ = 0.0;
  double progress_current_time_secs_ = 0.0;

  /// progress_bar_lines_count_ - The number of lines in progress bar to be printed out.
  ///  ====== Playback Progress ======
  ///  [0.000000000] Duration 0.00/0.00 [R]
  static const size_t progress_bar_lines_count_ = 2;
};
}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_PROGRESS_BAR_IMPL_HPP_
