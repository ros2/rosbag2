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

#ifndef FAKERECORDER_HPP_
#define FAKERECORDER_HPP_


#include <iostream>
#include <memory>
#include <string>
#include "rosbag2_keyboard_handler/KeyboardHandler.hpp"

class FakeRecorder
{
public:
  const FakeRecorder & operator=(const FakeRecorder &) = delete;

  FakeRecorder(const FakeRecorder &) = delete;

  virtual ~FakeRecorder()
  {
//    std::cout << "FakeRecorder() dtor" << std::endl;
  }

  static std::shared_ptr<FakeRecorder> create()
  {
    auto recorder_shared_ptr = std::shared_ptr<FakeRecorder>(new FakeRecorder);
    recorder_shared_ptr->weak_self_ = recorder_shared_ptr;
    return recorder_shared_ptr;
  }

  void register_callbacks(KeyboardHandler & keyboard_handler)
  {
    auto callback = [recorder_weak_ptr = weak_self_](const std::string & key_code) {
        auto RecorderSharedPtr = recorder_weak_ptr.lock();
        if (RecorderSharedPtr) {
          RecorderSharedPtr->callback_func(key_code);
        } else {
          std::cout << "Object for assigned callback FakeRecorder() was deleted" << std::endl;
        }
      };
    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KEY_CODE_CURSOR_UP);
  }

private:
  FakeRecorder()
  {
//    std::cout << "FakeRecorder() ctor" << std::endl;
  }

  virtual void callback_func(const std::string & keycode)
  {
    if (keycode.length() == 3 &&
      keycode[0] == KeyboardHandler::KEY_CODE_CURSOR_UP[0] &&
      keycode[1] == KeyboardHandler::KEY_CODE_CURSOR_UP[1])
    {
      switch (keycode[2]) {
        case KeyboardHandler::KEY_CODE_CURSOR_UP[2]:
          std::cout << "FakeRecorder callback with keycode = KEY_CODE_CURSOR_UP" << std::endl;
          break;
        case KeyboardHandler::KEY_CODE_CURSOR_DOWN[2]:
          std::cout << "FakeRecorder callback with keycode = KEY_CODE_CURSOR_DOWN" << std::endl;
          break;
        case KeyboardHandler::KEY_CODE_CURSOR_FORWARD[2]:
          std::cout << "FakeRecorder callback with keycode = KEY_CODE_CURSOR_FORWARD" << std::endl;
          break;
        case KeyboardHandler::KEY_CODE_CURSOR_BACK[2]:
          std::cout << "FakeRecorder callback with keycode = KEY_CODE_CURSOR_BACK" << std::endl;
          break;
        default:
          std::cout << "FakeRecorder callback with keycode = " << keycode << std::endl;
          break;
      }
    } else {
      std::cout << "FakeRecorder callback with keycode = " << keycode << std::endl;
    }
    counter_++;
  }

  std::weak_ptr<FakeRecorder> weak_self_;
  int counter_ = 0;
};

#endif  // FAKERECORDER_HPP_
