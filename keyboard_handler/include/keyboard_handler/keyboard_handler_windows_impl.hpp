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
#include "keyboard_handler/visibility_control.hpp"
#include "keyboard_handler_base.hpp"

class KeyboardHandlerWindowsImpl : public KeyboardHandlerBase
{
public:
  struct WinKeyCode
  {
    int first;
    int second;

    static constexpr int NOT_A_KEY = -1;

    bool operator==(const WinKeyCode & rhs) const
    {
      if (first == rhs.first && second == rhs.second) {
        return true;
      } else {
        return false;
      }
    }

    bool operator!=(const WinKeyCode & rhs) const
    {
      return !operator==(rhs);
    }
  };

  KEYBOARD_HANDLER_PUBLIC
  KeyboardHandlerWindowsImpl();

  KEYBOARD_HANDLER_PUBLIC
  virtual ~KeyboardHandlerWindowsImpl();

  KEYBOARD_HANDLER_PUBLIC
  KeyboardHandlerBase::KeyCode win_key_code_to_enum(const WinKeyCode & win_key_code) const;

  KEYBOARD_HANDLER_PUBLIC
  WinKeyCode enum_key_code_to_win_code(KeyboardHandlerBase::KeyCode key_code) const;

protected:
  // The specialized hash function for `unordered_map` keys
  struct win_key_code_hash_fn
  {
    std::size_t operator()(const WinKeyCode & key_code) const
    {
      return std::hash<int>()(key_code.first) ^ (std::hash<int>()(key_code.second) << 1);
    }
  };

  struct KeyMap
  {
    KeyboardHandlerBase::KeyCode inner_code;
    WinKeyCode win_key_code;
  };

  static const KeyMap DEFAULT_STATIC_KEY_MAP[];
  static const size_t STATIC_KEY_MAP_LENGTH;

  std::thread key_handler_thread_;
  std::atomic_bool exit_;
  std::unordered_map<WinKeyCode, KeyCode, win_key_code_hash_fn> key_codes_map_;
};

#endif  // KEYBOARD_HANDLER__KEYBOARD_HANDLER_WINDOWS_IMPL_HPP_
