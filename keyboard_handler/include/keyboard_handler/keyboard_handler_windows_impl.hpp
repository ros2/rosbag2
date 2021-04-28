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

#ifndef KEYBOARD_HANDLER__KEYBOARD_HANDLER_WINDOWS_IMPL_HPP_
#define KEYBOARD_HANDLER__KEYBOARD_HANDLER_WINDOWS_IMPL_HPP_

#include <atomic>
#include <thread>
#include <unordered_map>
#include <stdexcept>
#include "keyboard_handler/visibility_control.hpp"
#include "keyboard_handler_base.hpp"

/// \brief Windows specific implementation of keyboard handler class.
class KeyboardHandlerWindowsImpl : public KeyboardHandlerBase
{
public:
  /// \brief Data type for representing key codes returning by _getch() function in response to
  /// the pressing keyboard keys.
  /// \details Windows OS could return up to two integer values in response to the pressing
  /// keyboard key combination. If Windows returning only one value it will be placed in `first`
  /// field and second field will be initialized with NOT_A_KEY value.
  struct WinKeyCode
  {
    int first;
    int second;

    /// \brief Predefined value which is corresponding to the invalid or empty value for
    /// WinKeyCode elements.
    static constexpr int NOT_A_KEY = -1;

    /// \brief Equality operator for comparison of two WinKeyCodes.
    /// \param rhs Value for comparison from right hand side.
    /// \return Returns true if two key codes are equal, otherwise false.
    bool operator==(const WinKeyCode & rhs) const
    {
      if (first == rhs.first && second == rhs.second) {
        return true;
      } else {
        return false;
      }
    }

    /// \brief Not equal operator for comparison of two WinKeyCodes.
    /// \param rhs Value for comparison from right hand side.
    /// \return Returns true if two key codes are not equal, otherwise false.
    bool operator!=(const WinKeyCode & rhs) const
    {
      return !operator==(rhs);
    }
  };

  /// \brief Default constructor
  KEYBOARD_HANDLER_PUBLIC
  KeyboardHandlerWindowsImpl();

  /// \brief Destructor
  KEYBOARD_HANDLER_PUBLIC
  virtual ~KeyboardHandlerWindowsImpl();

  /// \brief Translates WinKeyCode to the internally defined KeyCode enum value.
  /// \param win_key_code Key codes returning by Windows OS in response to the pressing keyboard
  /// keys.
  /// \return Returns KeyboardHandlerBase::KeyCode enum value corresponding to the WinKeyCode in
  /// inner lookup table. If KeyCode enum value not found in the lookup table will return
  /// KeyboardHandlerBase::KeyCode::UNKNOWN value.
  KEYBOARD_HANDLER_PUBLIC
  KeyboardHandlerBase::KeyCode win_key_code_to_enum(const WinKeyCode & win_key_code) const;

  /// \brief Translates internally defined KeyCode enum value to the corresponding
  /// WinKeyCode registered in inner look up table.
  /// \param key_code Internally defined KeyCode enum value.
  /// \return Returns WinKeyCode value corresponding to the input KeyCode enum value in the
  /// inner lookup table. If corresponding WinKeyCode value not found in inner LUT will return
  /// WinKeyCode with NOT_A_KEY values.
  KEYBOARD_HANDLER_PUBLIC
  WinKeyCode enum_key_code_to_win_code(KeyboardHandlerBase::KeyCode key_code) const;

protected:
  /// \brief Specialized hash function for `unordered_map` with WinKeyCode keys
  struct win_key_code_hash_fn
  {
    std::size_t operator()(const WinKeyCode & key_code) const
    {
      return std::hash<int>()(key_code.first) ^ (std::hash<int>()(key_code.second) << 1);
    }
  };

  /// \brief Data type for mapping KeyCode enum value to the expecting sequence of key codes
  /// returning by _getch() in response to the pressing keyboard keys.
  struct KeyMap
  {
    KeyboardHandlerBase::KeyCode inner_code;
    WinKeyCode win_key_code;
  };

  /// \brief Default statically defined lookup table for corresponding KeyCode enum value and
  /// expecting sequence of key codes returning by _getch() in response to the pressing keyboard
  /// keys.
  static const KeyMap DEFAULT_STATIC_KEY_MAP[];

  /// \brief Length of DEFAULT_STATIC_KEY_MAP measured in number of elements.
  static const size_t STATIC_KEY_MAP_LENGTH;

private:
  std::thread key_handler_thread_;
  std::atomic_bool exit_;
  std::unordered_map<WinKeyCode, KeyCode, win_key_code_hash_fn> key_codes_map_;
  std::exception_ptr thread_exception_ptr = nullptr;
};

#endif  // KEYBOARD_HANDLER__KEYBOARD_HANDLER_WINDOWS_IMPL_HPP_
