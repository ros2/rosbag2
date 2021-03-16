// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2020, TNG Technology Consulting GmbH.
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

#include "rosbag2_transport/rosbag2_transport.hpp"

#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

#include "rosbag2_transport/logging.hpp"

#include "rosbag2_transport/player.hpp"
#include "recorder.hpp"
#include "rosbag2_transport/rosbag2_node.hpp"

namespace rosbag2_transport
{

#define CSI 0x5b1b  // Control Sequence Introducer
#define KEYCODE_CURSOR_BACK 'D'
#define KEYCODE_CURSOR_FORWARD 'C'
#define KEYCODE_CURSOR_UP 'A'
#define KEYCODE_CURSOR_DOWN 'B'
#define KEYCODE_SPACE ' '

struct termios old_term_settings {};

void quit(int sig)
{
  (void)sig;
  // Restore buffer mode for stdin
  if (tcsetattr(fileno(stdin), TCSANOW, &old_term_settings) == -1) {
    exit(EXIT_FAILURE);
  }
  exit(EXIT_SUCCESS);
}

Rosbag2Transport::Rosbag2Transport()
: reader_(std::make_shared<rosbag2_cpp::Reader>(
      std::make_unique<rosbag2_cpp::readers::SequentialReader>())),
  writer_(std::make_shared<rosbag2_cpp::Writer>(
      std::make_unique<rosbag2_cpp::writers::SequentialWriter>()))
{}

Rosbag2Transport::Rosbag2Transport(
  std::shared_ptr<rosbag2_cpp::Reader> reader,
  std::shared_ptr<rosbag2_cpp::Writer> writer)
: reader_(std::move(reader)), writer_(std::move(writer)) {}

void Rosbag2Transport::init()
{
  rclcpp::init(0, nullptr);
}

void Rosbag2Transport::shutdown()
{
  rclcpp::shutdown();
}

void Rosbag2Transport::record(
  const rosbag2_storage::StorageOptions & storage_options, const RecordOptions & record_options)
{
  try {
    writer_->open(
      storage_options, {rmw_get_serialization_format(), record_options.rmw_serialization_format});

    auto transport_node = setup_node(record_options.node_prefix);

    Recorder recorder(writer_, transport_node);
    recorder.record(record_options);
  } catch (std::runtime_error & e) {
    ROSBAG2_TRANSPORT_LOG_ERROR("Failed to record: %s", e.what());
  }
}

std::shared_ptr<Rosbag2Node> Rosbag2Transport::setup_node(
  std::string node_prefix,
  const std::vector<std::string> & topic_remapping_options)
{
  if (!transport_node_) {
    auto node_options = rclcpp::NodeOptions().arguments(topic_remapping_options);
    transport_node_ = std::make_shared<Rosbag2Node>(node_prefix + "_rosbag2", node_options);
  }
  return transport_node_;
}

void Rosbag2Transport::play(
  const rosbag2_storage::StorageOptions & storage_options, const PlayOptions & play_options)
{
  bool handle_key_press = false;
  const int fd = fileno(stdin);
  // Check if we can handle key press during playback
  if (!isatty(fd)) {
    // If stdin is not a real terminal (redirected to text file or pipe ) can't do much here
    // with keyboard handling. Note: this is usually happen when running under the gtest.
    ROSBAG2_TRANSPORT_LOG_WARN("stdin is not a terminal device. Keyboard handling disabled.");
    handle_key_press = false;
  } else {
    struct termios new_term_settings;
    if (tcgetattr(fd, &old_term_settings) == -1) {
      throw std::runtime_error("Error in tcgetattr(). errno = " + std::to_string(errno));
    }

    signal(SIGINT, quit);  // Setup signal handler to return terminal in original (buffered) mode
    // in case of abnormal program termination.

    new_term_settings = old_term_settings;
    // Set stdin to unbuffered mode for reading directly from the stdin.
    // Disable canonical input and disable echo.
    new_term_settings.c_lflag &= ~(ICANON | ECHO);
    new_term_settings.c_cc[VMIN] = 0;   // 0 means purely timeout driven readout
    new_term_settings.c_cc[VTIME] = 1;  // Wait maximum for 0.1 sec since start of the read() call.

    if (tcsetattr(fd, TCSANOW, &new_term_settings) == -1) {
      throw std::runtime_error("Error in tcsetattr(). errno = " + std::to_string(errno));
    }
    ROSBAG2_TRANSPORT_LOG_INFO(
      "Press `Space bar` for pause/resume, `Cursor Up/Down` to "
      "increase/decrease playback rate and `Cursor Forward` for playing next message.");
    handle_key_press = true;
  }

  try {
    auto transport_node =
      setup_node(play_options.node_prefix, play_options.topic_remapping_options);
    Player player(reader_, transport_node);
    do {
      reader_->open(storage_options, {"", rmw_get_serialization_format()});
      if (!handle_key_press) {
        player.play(play_options);
      } else {
        std::future<void> play_future_result =
          std::async(std::launch::async, &Player::play, &player, play_options);

        static constexpr size_t BUFF_LEN = 10;
        char buff[BUFF_LEN] = {0};

        auto play_finished = [&]() {
            if (play_future_result.valid() &&
              play_future_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
              play_future_result.get();
              return true;
            } else {
              return false;
            }
          };

        do {
          int read_bytes = read(fd, buff, BUFF_LEN);
          if (read_bytes < 0 && errno != EAGAIN) {
            throw std::runtime_error("Error in read(). errno = " + std::to_string(errno));
          }
          if (read_bytes == 0) {
            // Do nothing. 0 means read() returned by timeout.
          } else if (read_bytes == 1) {
            // buff[0] == alphanumerical key pressed
            if (buff[0] == KEYCODE_SPACE) {
              player.pause_resume();
            }
          } else if (read_bytes >= 3) {
            if (*reinterpret_cast<uint16_t *>(&buff[0]) == CSI) {  // Control Sequence Introducer
              switch (buff[2]) {
                case KEYCODE_CURSOR_FORWARD:
                  player.play_next();
                  ROSBAG2_TRANSPORT_LOG_INFO("Played next message");
                  break;
                case KEYCODE_CURSOR_UP: {
                    auto playback_rate = player.get_playback_rate() + 0.1;
                    player.set_playback_rate(playback_rate);
                    ROSBAG2_TRANSPORT_LOG_INFO("Increase playback rate to: %.4f", playback_rate);
                    break;
                  }
                case KEYCODE_CURSOR_DOWN: {
                    auto playback_rate = player.get_playback_rate() - 0.1;
                    player.set_playback_rate(playback_rate);
                    ROSBAG2_TRANSPORT_LOG_INFO(
                      "Decrease playback rate to: %.4f", playback_rate);
                    break;
                  }
                default:
                  break;
              }
            }
          }
        } while (!play_finished());
      }
    } while (rclcpp::ok() && play_options.loop);
  } catch (std::runtime_error & e) {
    ROSBAG2_TRANSPORT_LOG_ERROR("Failed to play: %s", e.what());
  }

  if (handle_key_press) {
    // Restore buffer mode for stdin
    if (tcsetattr(fd, TCSANOW, &old_term_settings) == -1) {
      throw std::runtime_error(
              "Error in tcsetattr old_term_settings. errno = " + std::to_string(errno));
    }
  }
}

}  // namespace rosbag2_transport
