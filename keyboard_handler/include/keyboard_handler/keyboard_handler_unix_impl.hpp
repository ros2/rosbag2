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

#ifndef KEYBOARD_HANDLER__KEYBOARD_HANDLER_UNIX_IMPL_HPP_
#define KEYBOARD_HANDLER__KEYBOARD_HANDLER_UNIX_IMPL_HPP_

#ifndef _WIN32
#include <string>
#include <unordered_map>
#include <atomic>
#include <thread>
#include <stdexcept>
#include "keyboard_handler/visibility_control.hpp"
#include "keyboard_handler_base.hpp"

/// \brief Unix (Posix) specific implementation of keyboard handler class.
class KeyboardHandlerUnixImpl : public KeyboardHandlerBase
{
public:
  using isattyFunction = std::function<int (int)>;
  using tcgetattrFunction = std::function<int (int, struct termios *)>;
  using tcsetattrFunction = std::function<int (int, int, const struct termios *)>;
  using readFunction = std::function<ssize_t(int, void *, size_t)>;

  /// \brief Default constructor
  KEYBOARD_HANDLER_PUBLIC
  KeyboardHandlerUnixImpl();

  /// \brief destructor
  KEYBOARD_HANDLER_PUBLIC
  virtual ~KeyboardHandlerUnixImpl();

  /// \brief Translates specified key press combination to the corresponding registered sequence of
  /// characters returning by terminal in response to the pressing keyboard keys.
  /// \param key_code Value from enum which corresponds to some predefined key press combination.
  /// \return Returns string with sequence of characters expecting to be returned by terminal.
  KEYBOARD_HANDLER_PUBLIC
  std::string get_terminal_sequence(KeyboardHandlerUnixImpl::KeyCode key_code);

protected:
  /// \brief Constructor with references to the system functions. Required for unit tests.
  /// \param read_fn Reference to the system read(int, void *, size_t) function
  /// \param isatty_fn Reference to the system isatty(int) function
  /// \param tcgetattr_fn Reference to the system tcgetattr(int, struct termios *) function
  /// \param tcsetattr_fn Reference to the system tcsetattr(int, int, const struct termios *)
  /// function
  KeyboardHandlerUnixImpl(
    const readFunction & read_fn,
    const isattyFunction & isatty_fn,
    const tcgetattrFunction & tcgetattr_fn,
    const tcsetattrFunction & tcsetattr_fn);

  /// \brief Data type for mapping KeyCode enum value to the expecting sequence of characters
  /// returning by terminal.
  struct KeyMap
  {
    KeyCode inner_code;
    const char * terminal_sequence;
  };

  /// \brief Default statically defined lookup table for corresponding KeyCode enum values and
  /// expecting sequence of characters returning by terminal.
  static const KeyMap DEFAULT_STATIC_KEY_MAP[];

  /// \brief Length of DEFAULT_STATIC_KEY_MAP  measured in number of elements.
  static const size_t STATIC_KEY_MAP_LENGTH;

private:
  std::thread key_handler_thread_;
  std::atomic_bool exit_;
  const int stdin_fd_;
  std::unordered_map<std::string, KeyCode> key_codes_map_;
  std::exception_ptr thread_exception_ptr{nullptr};
};

#endif  // #ifndef _WIN32
#endif  // KEYBOARD_HANDLER__KEYBOARD_HANDLER_UNIX_IMPL_HPP_
