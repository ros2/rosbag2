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

#include "rclcpp/logger.hpp"
#include "rcutils/time.h"
#include "rosbag2_transport/player_progress_bar.hpp"
#include "player_progress_bar_impl.hpp"

namespace rosbag2_transport
{

PlayerProgressBar::PlayerProgressBar(
  std::ostream & output_stream,
  rcutils_time_point_value_t starting_time,
  rcutils_time_point_value_t ending_time,
  int32_t progress_bar_update_rate,
  uint32_t progress_bar_separation_lines)
{
  pimpl_ = std::make_unique<PlayerProgressBarImpl>(
    output_stream, starting_time, ending_time,
    progress_bar_update_rate, progress_bar_separation_lines);
}

PlayerProgressBar::~PlayerProgressBar() = default;

void PlayerProgressBar::print_help_str() const
{
  pimpl_->print_help_str();
}

void PlayerProgressBar::update_with_limited_rate(
  const PlayerStatus & status,
  const rcutils_time_point_value_t & timestamp)
{
  pimpl_->update_with_limited_rate(status, timestamp);
}

void PlayerProgressBar::update(
  const PlayerStatus & status,
  const rcutils_time_point_value_t & timestamp)
{
  pimpl_->update(status, timestamp);
}

PlayerProgressBar::PlayerStatus PlayerProgressBar::get_player_status()
{
  return pimpl_->current_player_status_.load();
}

}  // namespace rosbag2_transport
