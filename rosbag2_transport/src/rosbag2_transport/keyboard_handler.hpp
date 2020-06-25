// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_TRANSPORT__KEYBOARD_HANDLER_HPP_
#define ROSBAG2_TRANSPORT__KEYBOARD_HANDLER_HPP_

#include <sys/select.h>

#if !defined(_MSC_VER)
  #include <termios.h>
  #include <unistd.h>
#else
  #include <windows.h>
#endif

namespace rosbag2_transport
{

class KeyboardHandler
{
public:
  KeyboardHandler();
  ~KeyboardHandler();

  void setup_terminal();
  void restore_terminal();
  int read_char_from_stdin();

private:
  bool terminal_modified_;
#if defined(_MSC_VER)
  HANDLE input_handle;
  DWORD stdin_set;
#else
  termios orig_flags_;
  fd_set stdin_fdset_;
#endif
  int maxfd_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__KEYBOARD_HANDLER_HPP_
