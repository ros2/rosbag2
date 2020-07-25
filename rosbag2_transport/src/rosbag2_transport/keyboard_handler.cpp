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

#include "keyboard_handler.hpp"

#include <stdio.h>

namespace rosbag2_transport
{

KeyboardHandler::KeyboardHandler()
: terminal_modified_(false)
{
}

KeyboardHandler::~KeyboardHandler()
{
  restore_terminal();
}

void KeyboardHandler::setup_terminal()
{
  if (terminal_modified_) {
    return;
  }

#if defined(_MSC_VER)
  input_handle = GetStdHandle(STD_INPUT_HANDLE);
  if (input_handle == INVALID_HANDLE_VALUE) {
    std::cout << "Failed to set up standard input handle." << std::endl;
    return;
  }
  if (!GetConsoleMode(input_handle, &stdin_set)) {
    std::cout << "Failed to save the console mode." << std::endl;
    return;
  }
  terminal_modified_ = true;
#else
  const int fd = fileno(stdin);
  termios flags;
  tcgetattr(fd, &orig_flags_);
  flags = orig_flags_;
  flags.c_lflag &= ~ICANON;      // set raw (unset canonical modes)
  flags.c_cc[VMIN] = 0;         // i.e. min 1 char for blocking, 0 chars for non-blocking
  flags.c_cc[VTIME] = 0;         // block if waiting for char
  tcsetattr(fd, TCSANOW, &flags);

  FD_ZERO(&stdin_fdset_);
  FD_SET(fd, &stdin_fdset_);
  maxfd_ = fd + 1;
  terminal_modified_ = true;
#endif
}

void KeyboardHandler::restore_terminal()
{
  if (!terminal_modified_) {
    return;
  }

#if defined(_MSC_VER)
  SetConsoleMode(input_handle, stdin_set);
#else
  const int fd = fileno(stdin);
  tcsetattr(fd, TCSANOW, &orig_flags_);
#endif
  terminal_modified_ = false;
}

int KeyboardHandler::read_char_from_stdin()
{
#if defined(__APPLE__)
  fd_set testfd;
  FD_COPY(&stdin_fdset_, &testfd);
#elif !defined(_MSC_VER)
  fd_set testfd = stdin_fdset_;
#endif

#if defined(_MSC_VER)
  DWORD events = 0;
  INPUT_RECORD input_record[1];
  DWORD input_size = 1;
  BOOL b = GetNumberOfConsoleInputEvents(input_handle, &events);
  if (b && events > 0) {
    b = ReadConsoleInput(input_handle, input_record, input_size, &events);
    if (b) {
      for (unsigned int i = 0; i < events; ++i) {
        if (input_record[i].EventType & KEY_EVENT & input_record[i].Event.KeyEvent.bKeyDown) {
          CHAR ch = input_record[i].Event.KeyEvent.uChar.AsciiChar;
          return ch;
        }
      }
    }
  }
  return EOF;
#else
  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  if (select(maxfd_, &testfd, NULL, NULL, &tv) <= 0) {
    return EOF;
  }
  return getc(stdin);
#endif
}

}  // namespace rosbag2_transport
