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
constexpr KeyboardHandlerBase::callback_handle_t KeyboardHandlerBase::invalid_handle;

KEYBOARD_HANDLER_PUBLIC
KeyboardHandlerBase::callback_handle_t KeyboardHandlerBase::add_key_press_callback(
  const callback_t & callback, KeyboardHandlerBase::KeyCode key_code)
{
  if (callback == nullptr || !is_init_succeed_) {
    return invalid_handle;
  }
  std::lock_guard<std::mutex> lk(callbacks_mutex_);
  callbacks_.emplace(key_code, callback_data{get_new_handle(), callback});
  return last_handle_;
}

KEYBOARD_HANDLER_PUBLIC
KeyboardHandlerBase::KeyCode & operator++(KeyboardHandlerBase::KeyCode & key_code)
{
  using KeyCode = KeyboardHandlerBase::KeyCode;
/* *INDENT-OFF* */
  key_code = static_cast<KeyCode>(static_cast<std::underlying_type_t<KeyCode>>(key_code) + 1);
/* *INDENT-ON* */
  return key_code;
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

KEYBOARD_HANDLER_PUBLIC
void KeyboardHandlerBase::delete_key_press_callback(const callback_handle_t & handle) noexcept
{
  for (auto it = callbacks_.begin(); it != callbacks_.end(); ++it) {
    if (it->second.handle == handle) {
      callbacks_.erase(it);
      return;
    }
  }
}

KeyboardHandlerBase::callback_handle_t KeyboardHandlerBase::get_new_handle()
{
  return ++last_handle_;
}
