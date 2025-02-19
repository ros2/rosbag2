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

#include <memory>
#include <string>

#include "rcutils/time.h"
#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{
class PlayerProgressBarImpl;

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

  /// PlayerProgressBar constructor
  /// \param output_stream Reference to output stream where progress bar and help information
  /// will be printed out. Could be std::cout, std::cerr or std::ostringstream for tests.
  /// \param starting_time Time stamp of the first message in the bag.
  /// \param ending_time Time stamp of the last message in the bag.
  /// \param progress_bar_update_rate The progress bar maximum update rate in times per second (Hz).
  /// If update rate equal 0 the progress bar will be disabled and will not output any information.
  /// If update rate less than 0 the progress bar will be updated for every update(..) and
  /// update_with_limited_rate(..) calls.
  /// \param progress_bar_separation_lines Number of separation lines to print in between the
  /// playback output and the progress bar.
  explicit PlayerProgressBar(
    std::ostream & output_stream,
    rcutils_time_point_value_t starting_time,
    rcutils_time_point_value_t ending_time,
    int32_t progress_bar_update_rate = 0,
    uint32_t progress_bar_separation_lines = 3);

  virtual ~PlayerProgressBar();

  /// \brief Prints help string about player progress bar.
  /// Expected to be printed once at the beginning.
  void print_help_str() const;

  /// \brief Updates progress bar with the specified timestamp and player status, taking into
  /// account the update rate set by the user.
  /// \note The function should be used for regular progress bar updates, for example
  /// after the publishing the next message.
  /// \warning This function is not thread safe and shall not be called concurrently from multiple
  /// threads.
  /// \param status The player status to be updated on progress bar.
  /// \param timestamp Timestamp of the last published message.
  void update_with_limited_rate(
    const PlayerStatus & status,
    const rcutils_time_point_value_t & timestamp);

  /// \brief Updates progress bar with the specified player status, irrespective to the update rate
  /// set by the user.
  /// \note The function should be called for extraordinary progress bar updates, for example
  /// when player changed its internal status or a log message is printed, and we want to 'redraw'
  /// the progress bar.
  /// \param status The player status to be updated on progress bar.
  /// \param timestamp Timestamp of the last published message. If timestamp is less than 0 the
  /// last published timestamp will be taken from the inner progress bar cache.
  void update(const PlayerStatus & status, const rcutils_time_point_value_t & timestamp = -1);

  /// \brief Getter function for current player status.
  /// \return Current player status.
  PlayerStatus get_player_status();

private:
  std::unique_ptr<PlayerProgressBarImpl> pimpl_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_PROGRESS_BAR_HPP_
