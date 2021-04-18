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

#ifndef ROSBAG2_KEYBOARD_HANDLER__KEYBOARDHANDLERBASE_HPP_
#define ROSBAG2_KEYBOARD_HANDLER__KEYBOARDHANDLERBASE_HPP_

#include <unordered_map>
#include <mutex>
#include <string>
#include "rosbag2_keyboard_handler/visibility_control.hpp"
#include "rosbag2_keyboard_handler/IKeyboardHandler.hpp"

class KeyboardHandlerBase : public IKeyboardHandler
{
public:
  KEYBOARD_HANDLER_PUBLIC
  bool add_key_press_callback(const callback_t & callback, const std::string & key_code) override
  {
    if (callback == nullptr || key_code.empty() || !is_init_succeed_) {
      return false;
    }
    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    callbacks_.emplace(key_code, callback);
    return true;
  }

protected:
  bool is_init_succeed_ = false;
  std::mutex callbacks_mutex_;
  std::unordered_multimap<std::string, callback_t> callbacks_;
};

#endif  // ROSBAG2_KEYBOARD_HANDLER__KEYBOARDHANDLERBASE_HPP_
