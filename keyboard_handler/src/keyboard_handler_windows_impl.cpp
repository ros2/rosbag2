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

#ifdef _WIN32
#include "keyboard_handler/keyboard_handler_windows_impl.hpp"

KeyboardHandlerWindowsImpl::KeyboardHandlerWindowsImpl()
: exit_(false)
{
  is_init_succeed_ = false;

  key_handler_thread_ = std::thread(
    [&]() {
      try {
        do {
          // TODO(morlov): Add windows specific implementation
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        } while (!exit_.load());
      } catch (...) {
        throw;
      }
    });
}

KeyboardHandlerWindowsImpl::~KeyboardHandlerWindowsImpl()
{
  exit_ = true;
  if (key_handler_thread_.joinable()) {
    key_handler_thread_.join();
  }
}
#endif  // #ifdef _WIN32
