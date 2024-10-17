// Copyright 2024 Nicola Loi.
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

#include "rcl/types.h"
#include "rclcpp/rclcpp.hpp"

namespace rosbag2_transport
{

class PlayerProgressBar final
{
public:
  explicit PlayerProgressBar(
    bool disable_progress_bar,
    rcutils_time_point_value_t bag_starting_time,
    rcutils_duration_value_t bag_duration)
  : disable_progress_bar_(disable_progress_bar),
    bag_starting_time_secs_(RCUTILS_NS_TO_S(static_cast<double>(bag_starting_time))),
    bag_duration_secs_(RCUTILS_NS_TO_S(static_cast<double>(bag_duration)))
  {
  }
  ~PlayerProgressBar()
  {
    if (!disable_progress_bar_) {
      printf("\n");
      fflush(stdout);
    }
  }
  inline void print_running_status(const rcutils_time_point_value_t current_time) const
  {
    if (!disable_progress_bar_) {
      print_status(current_time, 'R');
    }
  }
  inline void print_paused_status(const rcutils_time_point_value_t current_time) const
  {
    if (!disable_progress_bar_) {
      print_status(current_time, 'P');
    }
  }
  inline void print_delayed_status(const rcutils_time_point_value_t current_time) const
  {
    if (!disable_progress_bar_) {
      print_status(current_time, 'D');
    }
  }
  std::string get_help_str() const
  {
    if (!disable_progress_bar_) {
      return "Bag Time and Duration [?]: ? = R running, P paused, D delayed";
    } else {
      return "Bag Time and Duration progress bar disabled.";
    }
  }

private:
  inline void print_status(const rcutils_time_point_value_t current_time, const char status) const
  {
    double current_time_secs = RCUTILS_NS_TO_S(static_cast<double>(current_time));
    double bag_progress_secs = current_time_secs - bag_starting_time_secs_;
    printf(" Bag Time %13.6f  Duration %.6f/%.6f [%c] \r",
      current_time_secs, bag_progress_secs, bag_duration_secs_, status);
    fflush(stdout);
  }
  bool disable_progress_bar_;
  const double bag_starting_time_secs_;
  const double bag_duration_secs_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_PROGRESS_BAR_HPP_
