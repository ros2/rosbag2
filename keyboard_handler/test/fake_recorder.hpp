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

#ifndef FAKE_RECORDER_HPP_
#define FAKE_RECORDER_HPP_

#include <iostream>
#include <memory>
#include <string>
#include "keyboard_handler/keyboard_handler.hpp"

class FakeRecorder
{
public:
  const FakeRecorder & operator=(const FakeRecorder &) = delete;
  FakeRecorder(const FakeRecorder &) = delete;
  virtual ~FakeRecorder() = default;

  static std::shared_ptr<FakeRecorder> create()
  {
    auto recorder_shared_ptr = std::shared_ptr<FakeRecorder>(new FakeRecorder);
    recorder_shared_ptr->weak_self_ = recorder_shared_ptr;
    return recorder_shared_ptr;
  }

  void register_callbacks(KeyboardHandler & keyboard_handler)
  {
    auto callback = [recorder_weak_ptr = weak_self_](KeyboardHandler::KeyCode key_code) {
        auto recorder_shared_ptr = recorder_weak_ptr.lock();
        if (recorder_shared_ptr) {
          recorder_shared_ptr->callback_func(key_code);
        } else {
          std::cout << "Object for assigned callback FakeRecorder() was deleted" << std::endl;
        }
      };
    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::CURSOR_UP);
  }

private:
  FakeRecorder() = default;

  virtual void callback_func(KeyboardHandler::KeyCode key_code)
  {
    using KeyCode = KeyboardHandler::KeyCode;
    switch (key_code) {
      case KeyCode::CURSOR_UP:
        std::cout << "FakeRecorder callback with key code = CURSOR_UP" << std::endl;
        break;
      case KeyCode::CURSOR_DOWN:
        std::cout << "FakeRecorder callback with key code = CURSOR_DOWN" << std::endl;
        break;
      case KeyCode::CURSOR_RIGHT:
        std::cout << "FakeRecorder callback with key code = CURSOR_RIGHT" << std::endl;
        break;
      case KeyCode::CURSOR_LEFT:
        std::cout << "FakeRecorder callback with key code = CURSOR_LEFT" << std::endl;
        break;
      default:
        std::cout << "FakeRecorder callback with key code = " << static_cast<int32_t>(key_code) <<
          std::endl;
        break;
    }
    counter_++;
  }

  std::weak_ptr<FakeRecorder> weak_self_;
  int counter_ = 0;
};

#endif  // FAKE_RECORDER_HPP_
