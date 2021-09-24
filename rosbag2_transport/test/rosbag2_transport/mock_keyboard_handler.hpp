// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_TRANSPORT__MOCK_KEYBOARD_HANDLER_HPP_
#define ROSBAG2_TRANSPORT__MOCK_KEYBOARD_HANDLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "keyboard_handler/keyboard_handler.hpp"

// press one specific key code one second at a time
class MockKeyboardHandler : public KeyboardHandler
{
public:
  MockKeyboardHandler() = default;

  void simulate_key_press(
    KeyboardHandler::KeyCode key_code,
    KeyboardHandler::KeyModifiers key_modifiers = KeyboardHandler::KeyModifiers::NONE)
  {
    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    auto range = callbacks_.equal_range(KeyAndModifiers{key_code, key_modifiers});
    for (auto it = range.first; it != range.second; ++it) {
      it->second.callback(key_code, key_modifiers);
    }
  }
};

#endif  // ROSBAG2_TRANSPORT__MOCK_KEYBOARD_HANDLER_HPP_
