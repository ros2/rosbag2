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


#include <gmock/gmock.h>

#include <iostream>
#include <thread>

#include "rosbag2_transport/player_progress_bar.hpp"

using namespace ::testing;          // NOLINT
using namespace rosbag2_transport;  // NOLINT

class TestPlayerProgressBar : public Test
{
public:
  std::string GenerateClearAndMoveCursorDownRegexString(uint32_t num_lines)
  {
    std::ostringstream oss;
    for (uint32_t i = 0; i < num_lines; i++) {
      // The ANSI control code "\033[2K" is used to clear an entire line in the terminal.
      // Cleanup current line and jump down to one new line with "\n"
      oss << "\033\\[2K\n";
    }
    return oss.str();
  }

  std::string GenerateMoveCursorUpRegexString(uint32_t num_lines)
  {
    static const size_t progress_bar_lines_count = 2;
    // ====== Playback Progress ======
    // [0.000000000] Duration 0.00/0.00 [R]
    std::ostringstream oss;
    // Move cursor up by a specific number of lines using "Cursor Up" '\033[<N>A' escape sequence
    oss << "\033\\[" << num_lines + progress_bar_lines_count << "A";
    return oss.str();
  }

  std::vector<PlayerProgressBar::PlayerStatus> status_list_ = {
    PlayerProgressBar::PlayerStatus::DELAYED,
    PlayerProgressBar::PlayerStatus::RUNNING,
    PlayerProgressBar::PlayerStatus::PAUSED,
    PlayerProgressBar::PlayerStatus::BURST,
    PlayerProgressBar::PlayerStatus::STOPPED
  };
};

TEST_F(TestPlayerProgressBar, default_ctor_dtor) {
  std::ostringstream oss;
  {
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0);
  }
}

TEST_F(TestPlayerProgressBar, can_dtor_after_output) {
  std::ostringstream oss;
  {
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0);
    progress_bar->print_help_str();
    progress_bar->update(PlayerProgressBar::PlayerStatus::DELAYED);
  }
}

TEST_F(TestPlayerProgressBar, print_help_str_with_zero_update_rate) {
  std::ostringstream oss;
  {
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, 0);
    progress_bar->print_help_str();
    EXPECT_EQ(oss.str(), "Progress bar disabled.\n");
  }
}

TEST_F(TestPlayerProgressBar, print_help_str_with_negative_update_rate) {
  {
    std::ostringstream oss;
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, -1);
    progress_bar->print_help_str();
    EXPECT_EQ(oss.str(),
      "Progress bar enabled for every message."
      "\nProgress bar [?]: [R]unning, [P]aused, [B]urst, [D]elayed, [S]topped\n");
  }
}

TEST_F(TestPlayerProgressBar, print_help_str_with_positive_update_rate) {
  {
    std::ostringstream oss;
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, 1);
    progress_bar->print_help_str();
    EXPECT_EQ(oss.str(),
      "Progress bar enabled at 1 Hz."
      "\nProgress bar [?]: [R]unning, [P]aused, [B]urst, [D]elayed, [S]topped\n");
  }
}

TEST_F(TestPlayerProgressBar, update_status_with_disabled_progress_bar) {
  {
    std::ostringstream oss;
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, 0);
    for (const auto & status : status_list_) {
      progress_bar->update(status);
      EXPECT_THAT(oss.str(), IsEmpty());
      oss.clear();
      oss.str("");
    }
  }
}

TEST_F(TestPlayerProgressBar, update_status_with_enabled_progress_bar) {
  {
    // Test Playback Progress header
    std::ostringstream oss;
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, 1);
    progress_bar->update(PlayerProgressBar::PlayerStatus::RUNNING);
    progress_bar->update(PlayerProgressBar::PlayerStatus::STOPPED);
    EXPECT_THAT(oss.str(), MatchesRegex(
      ".*====== Playback Progress ======"
      "\n.*\\[R\\]"
      ".*====== Playback Progress ======"
      "\n.*\\[S\\].*"));
  }

  {
    // Test status update with update rate 1 Hz
    std::ostringstream oss;
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, 1);
    progress_bar->update(PlayerProgressBar::PlayerStatus::RUNNING);
    progress_bar->update(PlayerProgressBar::PlayerStatus::STOPPED);
    EXPECT_THAT(oss.str(), MatchesRegex(
      ".*\\[R\\].*\n.*\\[S\\].*"));
  }

  {
    // Test status update with update rate 10 Hz
    std::ostringstream oss;
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, 10);
    progress_bar->update(PlayerProgressBar::PlayerStatus::RUNNING);
    progress_bar->update(PlayerProgressBar::PlayerStatus::STOPPED);
    EXPECT_THAT(oss.str(), MatchesRegex(
      ".*\\[R\\].*\n.*\\[S\\].*"));
  }

  {
    // Test status update on all statuses
    std::ostringstream oss;
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, 1);
    for (const auto & status : status_list_) {
      progress_bar->update(PlayerProgressBar::PlayerStatus::RUNNING);
      progress_bar->update(status);
      std::string status_str(1, static_cast<char>(status));
      EXPECT_THAT(oss.str(), MatchesRegex(
        ".*\\[R\\].*\n.*\\[" + status_str + "\\].*"));
      oss.clear();
      oss.str("");
    }
  }
}

TEST_F(TestPlayerProgressBar, update_status_with_separation_lines) {
  {
    // Test status update with 2 separation lines
    std::ostringstream oss;
    uint32_t num_separation_lines = 2;
    std::string expected_pre_line_separator =
      GenerateClearAndMoveCursorDownRegexString(num_separation_lines);
    std::string expected_post_line_separator =
      GenerateMoveCursorUpRegexString(num_separation_lines);
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, 1, num_separation_lines);
    progress_bar->update(PlayerProgressBar::PlayerStatus::DELAYED);
    progress_bar->update(PlayerProgressBar::PlayerStatus::RUNNING);
    EXPECT_THAT(oss.str(), MatchesRegex(
      expected_pre_line_separator +
      ".*\\[D\\].*" + expected_post_line_separator +
      expected_pre_line_separator +
      ".*\\[R\\].*" + expected_post_line_separator));
  }

  {
    // Test status update with 5 separation lines
    std::ostringstream oss;
    uint32_t num_separation_lines = 5;
    std::string expected_pre_line_separator =
      GenerateClearAndMoveCursorDownRegexString(num_separation_lines);
    std::string expected_post_line_separator =
      GenerateMoveCursorUpRegexString(num_separation_lines);
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, 1, num_separation_lines);
    progress_bar->update(PlayerProgressBar::PlayerStatus::DELAYED);
    progress_bar->update(PlayerProgressBar::PlayerStatus::RUNNING);
    progress_bar->update(PlayerProgressBar::PlayerStatus::PAUSED);
    EXPECT_THAT(oss.str(), MatchesRegex(
      expected_pre_line_separator +
      ".*\\[D\\].*" + expected_post_line_separator +
      expected_pre_line_separator +
      ".*\\[R\\].*" + expected_post_line_separator +
      expected_pre_line_separator +
      ".*\\[P\\].*" + expected_post_line_separator));
  }

  {
    // Test status update with 5 separation lines and external log messages
    std::ostringstream oss;
    uint32_t num_separation_lines = 5;
    std::string expected_pre_line_separator =
      GenerateClearAndMoveCursorDownRegexString(num_separation_lines);
    std::string expected_post_line_separator =
      GenerateMoveCursorUpRegexString(num_separation_lines);
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, 1, num_separation_lines);
    progress_bar->update(PlayerProgressBar::PlayerStatus::DELAYED);
    oss << "log msg 1\n";
    oss << "log msg 2\n";
    oss << "log msg 3\n";
    oss << "log msg 4\n";
    progress_bar->update(PlayerProgressBar::PlayerStatus::RUNNING);
    oss << "log msg 5\n";
    oss << "log msg 6\n";
    progress_bar->update(PlayerProgressBar::PlayerStatus::PAUSED);
    EXPECT_THAT(oss.str(), MatchesRegex(
      expected_pre_line_separator +
      ".*\\[D\\].*" + expected_post_line_separator +
      "log msg 1\nlog msg 2\nlog msg 3\nlog msg 4\n" +
      expected_pre_line_separator +
      ".*\\[R\\].*" + expected_post_line_separator +
      "log msg 5\nlog msg 6\n" +
      expected_pre_line_separator +
      ".*\\[P\\].*" + expected_post_line_separator));
  }
}

TEST_F(TestPlayerProgressBar, update_status_without_separation_lines) {
  std::ostringstream oss;
  {
    // Test status update without separation lines
    uint32_t num_separation_lines = 0;
    std::string expected_pre_line_separator =
      GenerateClearAndMoveCursorDownRegexString(num_separation_lines);
    std::string expected_post_line_separator =
      GenerateMoveCursorUpRegexString(num_separation_lines);
    auto progress_bar = std::make_unique<PlayerProgressBar>(oss, 0, 0, 1, num_separation_lines);
    progress_bar->update(PlayerProgressBar::PlayerStatus::DELAYED);
    EXPECT_THAT(oss.str(), MatchesRegex(
      expected_pre_line_separator +
      ".*\\[D\\].*" + expected_post_line_separator));

    progress_bar->update(PlayerProgressBar::PlayerStatus::RUNNING);
    EXPECT_THAT(oss.str(), MatchesRegex(
      expected_pre_line_separator +
      ".*\\[D\\].*" + expected_post_line_separator +
      expected_pre_line_separator +
      ".*\\[R\\].*" + expected_post_line_separator));
  }
}

TEST_F(TestPlayerProgressBar, update_with_limited_rate_respect_update_rate) {
  rcutils_time_point_value_t starting_time = 1e18;  // nanoseconds
  rcutils_time_point_value_t ending_time = starting_time + 5e9;
  {
    // Test if update rate of the progress bar is respected
    std::ostringstream oss;
    int32_t update_rate = 3;
    uint64_t update_period_ns = 1e9 / update_rate;
    auto progress_bar = std::make_unique<PlayerProgressBar>(
      oss, starting_time, ending_time, update_rate, 0);
    std::vector<rcutils_time_point_value_t> timestamps;
    timestamps.push_back(starting_time);  // 1st update progress bar
    // add timestamps relative to 1st update progress bar
    timestamps.push_back(timestamps.at(0) + 0.3 * update_period_ns);  // skip update
    timestamps.push_back(timestamps.at(0) + 0.5 * update_period_ns);  // skip update
    timestamps.push_back(timestamps.at(0) + 1.3 * update_period_ns);  // 2nd update
    // add timestamps relative to 2nd update progress bar
    timestamps.push_back(timestamps.at(3) + 0.3 * update_period_ns);  // skip update
    timestamps.push_back(timestamps.at(3) + 1.3 * update_period_ns);  // 3rd update

    progress_bar->update_with_limited_rate(
      timestamps[0], PlayerProgressBar::PlayerStatus::RUNNING);
    for (size_t i = 1; i < timestamps.size(); ++i) {
      {
        std::this_thread::sleep_for(
          std::chrono::nanoseconds(timestamps[i] - timestamps[i - 1]));
      }
      progress_bar->update_with_limited_rate(
        timestamps[i], PlayerProgressBar::PlayerStatus::RUNNING);
    }
    // Check if the progress bar is updated at the correct 3 timestamps
    EXPECT_THAT(oss.str(), MatchesRegex(
      ".*\\[1000000000\\.000000000\\] Duration 0\\.00/5\\.00"
      ".*\\[1000000000\\.4333.*\\] Duration 0\\.43/5\\.00"
      ".*\\[1000000000\\.8666.*\\] Duration 0\\.87/5\\.00.*"));
  }
}

TEST_F(TestPlayerProgressBar, update_with_limited_rate_update_progress) {
  // TODO(someone): Add content for this test
}

TEST_F(TestPlayerProgressBar, update_with_limited_rate_with_negative_update_rate) {
  rcutils_time_point_value_t starting_time = 1e18;  // nanoseconds
  rcutils_time_point_value_t ending_time = starting_time + 5e9;
  int32_t update_rate = -1;
  {
    // Test if progress bar is updated every time
    std::ostringstream oss;
    auto progress_bar = std::make_unique<PlayerProgressBar>(
      oss, starting_time, ending_time, update_rate, 0);
    rcutils_time_point_value_t timestamp_1 = starting_time;
    rcutils_time_point_value_t timestamp_2 = starting_time + 1e6;
    rcutils_time_point_value_t timestamp_3 = starting_time + 2e6;
    progress_bar->update_with_limited_rate(timestamp_1, PlayerProgressBar::PlayerStatus::RUNNING);
    progress_bar->update_with_limited_rate(timestamp_2, PlayerProgressBar::PlayerStatus::STOPPED);
    progress_bar->update_with_limited_rate(timestamp_3, PlayerProgressBar::PlayerStatus::RUNNING);
    EXPECT_THAT(oss.str(), MatchesRegex(
      ".*\\[1000000000\\.000000000\\] Duration 0\\.00/5\\.00"
      ".*\\[1000000000\\.0009.*\\] Duration 0\\.00/5\\.00"
      ".*\\[1000000000\\.0019.*\\] Duration 0\\.00/5\\.00.*"));
  }

  {
    // Test if progress bar is updated every time
    std::ostringstream oss;
    auto progress_bar = std::make_unique<PlayerProgressBar>(
      oss, starting_time, ending_time, update_rate, 0);
    rcutils_time_point_value_t timestamp_1 = starting_time;
    rcutils_time_point_value_t timestamp_2 = starting_time + 1e9;
    rcutils_time_point_value_t timestamp_3 = starting_time + 2e9;
    progress_bar->update_with_limited_rate(timestamp_1, PlayerProgressBar::PlayerStatus::RUNNING);
    progress_bar->update_with_limited_rate(timestamp_2, PlayerProgressBar::PlayerStatus::STOPPED);
    progress_bar->update_with_limited_rate(timestamp_3, PlayerProgressBar::PlayerStatus::RUNNING);
    EXPECT_THAT(oss.str(), MatchesRegex(
      ".*\\[1000000000\\.000000000\\] Duration 0\\.00/5\\.00"
      ".*\\[1000000001\\.000000000\\] Duration 1\\.00/5\\.00"
      ".*\\[1000000002\\.000000000\\] Duration 2\\.00/5\\.00.*"));
  }
}

TEST_F(TestPlayerProgressBar, update_with_limited_rate_with_zero_update_rate) {
  rcutils_time_point_value_t starting_time = 1e18;  // nanoseconds
  rcutils_time_point_value_t ending_time = starting_time + 5e9;
  int32_t update_rate = 0;
  {
    // Test if progress bar is not updated
    std::ostringstream oss;
    auto progress_bar = std::make_unique<PlayerProgressBar>(
      oss, starting_time, ending_time, update_rate);
    rcutils_time_point_value_t timestamp_1 = starting_time;
    rcutils_time_point_value_t timestamp_2 = starting_time + 1e6;
    rcutils_time_point_value_t timestamp_3 = starting_time + 2e6;
    progress_bar->update_with_limited_rate(timestamp_1, PlayerProgressBar::PlayerStatus::DELAYED);
    progress_bar->update_with_limited_rate(timestamp_2, PlayerProgressBar::PlayerStatus::RUNNING);
    progress_bar->update_with_limited_rate(timestamp_3, PlayerProgressBar::PlayerStatus::RUNNING);
    EXPECT_THAT(oss.str(), IsEmpty());
  }

  {
    // Test if progress bar is not updated
    std::ostringstream oss;
    auto progress_bar = std::make_unique<PlayerProgressBar>(
      oss, starting_time, ending_time, update_rate);
    rcutils_time_point_value_t timestamp_1 = starting_time;
    rcutils_time_point_value_t timestamp_2 = starting_time + 1e9;
    rcutils_time_point_value_t timestamp_3 = starting_time + 2e9;
    progress_bar->update_with_limited_rate(timestamp_1, PlayerProgressBar::PlayerStatus::DELAYED);
    progress_bar->update_with_limited_rate(timestamp_2, PlayerProgressBar::PlayerStatus::RUNNING);
    progress_bar->update_with_limited_rate(timestamp_3, PlayerProgressBar::PlayerStatus::RUNNING);
    EXPECT_THAT(oss.str(), IsEmpty());
  }
}

TEST_F(TestPlayerProgressBar, update_with_limited_rate_with_zero_timestamp) {
  // TODO(someone): Add content for this test
}
