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

#ifndef FAKEPLAYER_HPP_
#define FAKEPLAYER_HPP_

#include <iostream>
#include <memory>
#include <string>
#include "rosbag2_keyboard_handler/KeyboardHandler.hpp"

class FakePlayer : public std::enable_shared_from_this<FakePlayer>
{
public:
  FakePlayer()
  {
//    std::cout << "FakePlayer() ctor" << std::endl;
  }

  void register_callbacks(KeyboardHandler & keyboard_handler)
  {
//    std::cout << "FakePlayer() register_callbacks" << std::endl;
    std::weak_ptr<FakePlayer> player_weak_ptr(shared_from_this());
    auto callback = [player_weak_ptr](const std::string & key_code) {
        auto player_shared_ptr = player_weak_ptr.lock();
        if (player_shared_ptr) {
          player_shared_ptr->callback_func(key_code);
        } else {
          std::cout << "Object for assigned callback FakePlayer() was deleted" << std::endl;
        }
      };

    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KEY_CODE_CURSOR_UP);
  }

  virtual ~FakePlayer()
  {
//    std::cout << "FakePlayer() dtor" << std::endl;
  }

private:
  void callback_func(const std::string & keycode)
  {
    if (keycode.length() == 3 &&
      keycode[0] == KeyboardHandler::KEY_CODE_CURSOR_UP[0] &&
      keycode[1] == KeyboardHandler::KEY_CODE_CURSOR_UP[1])
    {
      switch (keycode[2]) {
        case KeyboardHandler::KEY_CODE_CURSOR_UP[2]:
          std::cout << "FakePlayer callback with keycode = KEY_CODE_CURSOR_UP" << std::endl;
          break;
        case KeyboardHandler::KEY_CODE_CURSOR_DOWN[2]:
          std::cout << "FakePlayer callback with keycode = KEY_CODE_CURSOR_DOWN" << std::endl;
          break;
        case KeyboardHandler::KEY_CODE_CURSOR_FORWARD[2]:
          std::cout << "FakePlayer callback with keycode = KEY_CODE_CURSOR_FORWARD" << std::endl;
          break;
        case KeyboardHandler::KEY_CODE_CURSOR_BACK[2]:
          std::cout << "FakePlayer callback with keycode = KEY_CODE_CURSOR_BACK" << std::endl;
          break;
        default:
          std::cout << "FakePlayer callback with keycode = " << keycode << std::endl;
          break;
      }
    } else {
      std::cout << "FakePlayer callback with keycode = " << keycode << std::endl;
    }
    counter_++;
  }

  int counter_ = 0;
};


#endif  // FAKEPLAYER_HPP_
