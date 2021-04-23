// Copyright 2021 Apex.AI, Inc. or its affiliates. All Rights Reserved.
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

#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#include <string>
#include <algorithm>
#include <csignal>
#include <iostream>
#include "keyboard_handler/keyboard_handler_unix_impl.hpp"

KEYBOARD_HANDLER_PUBLIC
std::string
KeyboardHandlerUnixImpl::get_terminal_sequence(KeyboardHandlerUnixImpl::KeyCode key_code)
{
  std::string ret_str{};
  for (const auto & it : key_codes_map_) {
    if (it.second == key_code) {
      return it.first;
    }
  }
  return ret_str;
}

namespace
{
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
}  // namespace

KeyboardHandlerUnixImpl::KeyboardHandlerUnixImpl()
: exit_(false), stdin_fd_(fileno(stdin))
{
  for (size_t i = 0; i < STATIC_KEY_MAP_LENGTH; i++) {
    key_codes_map_.emplace(
      DEFAULT_STATIC_KEY_MAP[i].terminal_sequence,
      DEFAULT_STATIC_KEY_MAP[i].inner_code);
  }

  // Check if we can handle key press from std input
  if (!isatty(stdin_fd_)) {
    // If stdin is not a real terminal (redirected to text file or pipe ) can't do much here
    // with keyboard handling.
    std::cerr << "stdin is not a terminal device. Keyboard handling disabled.";
    return;
  }

  struct termios new_term_settings;
  if (tcgetattr(stdin_fd_, &old_term_settings) == -1) {
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

  if (tcsetattr(stdin_fd_, TCSANOW, &new_term_settings) == -1) {
    throw std::runtime_error("Error in tcsetattr(). errno = " + std::to_string(errno));
  }
  is_init_succeed_ = true;

  key_handler_thread_ = std::thread(
    [&]() {
      try {
        static constexpr size_t BUFF_LEN = 10;
        char buff[BUFF_LEN] = {0};
        do {
          ssize_t read_bytes = read(stdin_fd_, buff, BUFF_LEN);
          if (read_bytes < 0 && errno != EAGAIN) {
            throw std::runtime_error("Error in read(). errno = " + std::to_string(errno));
          }

          if (read_bytes == 0) {
            // Do nothing. 0 means read() returned by timeout.
          } else {  // read_bytes > 0
            buff[std::min(BUFF_LEN - 1, static_cast<size_t>(read_bytes))] = '\0';
            KeyCode pressed_key_code = KeyCode::UNKNOWN;
            std::lock_guard<std::mutex> lk(callbacks_mutex_);
            auto key_map_it = key_codes_map_.find(buff);
            if (key_map_it != key_codes_map_.end()) {
              pressed_key_code = key_map_it->second;
            }

            auto range = callbacks_.equal_range(pressed_key_code);
            for (auto it = range.first; it != range.second; ++it) {
              it->second(it->first);
            }
          }
        } while (!exit_.load());
      } catch (...) {
        // Restore buffer mode for stdin
        if (tcsetattr(stdin_fd_, TCSANOW, &old_term_settings) == -1) {
          throw std::runtime_error(
            "Error in tcsetattr old_term_settings. errno = " + std::to_string(errno));
        }
        throw;
      }
      // Restore buffer mode for stdin
      if (tcsetattr(stdin_fd_, TCSANOW, &old_term_settings) == -1) {
        throw std::runtime_error(
          "Error in tcsetattr old_term_settings. errno = " + std::to_string(errno));
      }
    });
}

KeyboardHandlerUnixImpl::~KeyboardHandlerUnixImpl()
{
  exit_ = true;
  if (key_handler_thread_.joinable()) {
    key_handler_thread_.join();
  }
}

#endif  // #ifndef _WIN32
