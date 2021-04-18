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

#ifndef ROSBAG2_KEYBOARD_HANDLER__IKEYBOARDHANDLER_HPP_
#define ROSBAG2_KEYBOARD_HANDLER__IKEYBOARDHANDLER_HPP_

#include <functional>
#include <string>

class IKeyboardHandler
{
public:
  using callback_t = std::function<void (const std::string &)>;

  static constexpr char KEY_CODE_CURSOR_UP[] = {27, 91, 'A', '\0'};
  static constexpr char KEY_CODE_CURSOR_DOWN[] = {27, 91, 'B', '\0'};
  static constexpr char KEY_CODE_CURSOR_FORWARD[] = {27, 91, 'C', '\0'};
  static constexpr char KEY_CODE_CURSOR_BACK[] = {27, 91, 'D', '\0'};
  static constexpr char KEY_CODE_SPACE[] = " ";

  virtual
  ~IKeyboardHandler() = default;

  virtual
  bool add_key_press_callback(const callback_t & callback, const std::string & key_code) = 0;
};

#endif  // ROSBAG2_KEYBOARD_HANDLER__IKEYBOARDHANDLER_HPP_
