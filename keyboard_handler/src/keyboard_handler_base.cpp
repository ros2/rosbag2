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

#include <string>
#include "keyboard_handler/keyboard_handler_base.hpp"

KEYBOARD_HANDLER_PUBLIC
bool KeyboardHandlerBase::add_key_press_callback(
  const callback_t & callback, KeyboardHandlerBase::KeyCode key_code)
{
  if (callback == nullptr || !is_init_succeed_) {
    return false;
  }
  std::lock_guard<std::mutex> lk(callbacks_mutex_);
  callbacks_.emplace(key_code, callback);
  return true;
}

KEYBOARD_HANDLER_PUBLIC
std::string enum_key_code_to_str(KeyboardHandlerBase::KeyCode key_code)
{
  for (auto & it : ENUM_KEY_TO_STR_MAP) {
    if (it.inner_code == key_code) {
      return it.str;
    }
  }
  return std::string();
}
