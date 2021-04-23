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

#ifndef FAKE_PLAYER_HPP_
#define FAKE_PLAYER_HPP_

#include <iostream>
#include <memory>
#include <string>
#include "keyboard_handler/keyboard_handler.hpp"

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
    auto callback = [player_weak_ptr](KeyboardHandler::KeyCode key_code) {
        auto player_shared_ptr = player_weak_ptr.lock();
        if (player_shared_ptr) {
          player_shared_ptr->callback_func(key_code);
        } else {
          std::cout << "Object for assigned callback FakePlayer() was deleted" << std::endl;
        }
      };

    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::CURSOR_UP);
  }

  virtual ~FakePlayer()
  {
//    std::cout << "FakePlayer() dtor" << std::endl;
  }

private:
  void callback_func(KeyboardHandler::KeyCode key_code)
  {
    using KeyCode = KeyboardHandler::KeyCode;
    switch (key_code) {
      case KeyCode::CURSOR_UP:
        std::cout << "FakePlayer callback with key code = CURSOR_UP" << std::endl;
        break;
      case KeyCode::CURSOR_DOWN:
        std::cout << "FakePlayer callback with key code = CURSOR_DOWN" << std::endl;
        break;
      case KeyCode::CURSOR_RIGHT:
        std::cout << "FakePlayer callback with key code = CURSOR_RIGHT" << std::endl;
        break;
      case KeyCode::CURSOR_LEFT:
        std::cout << "FakePlayer callback with key code = CURSOR_LEFT" << std::endl;
        break;
      default:
        std::cout << "FakePlayer callback with key code = " << static_cast<int32_t>(key_code) <<
          std::endl;
        break;
    }
    counter_++;
  }

  int counter_ = 0;
};


#endif  // FAKE_PLAYER_HPP_
