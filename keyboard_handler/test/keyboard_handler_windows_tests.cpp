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
#include <string>
#include <memory>
#include <algorithm>
#include "gmock/gmock.h"
#include "fake_recorder.hpp"
#include "fake_player.hpp"
#include "keyboard_handler/keyboard_handler_windows_impl.hpp"

using ::testing::Return;
using ::testing::Eq;
using ::testing::StrEq;
using ::testing::AtLeast;
using ::testing::_;
using ::testing::NiceMock;

class MockKeyboardHandler : public KeyboardHandlerWindowsImpl
{
public:
  size_t get_number_of_registered_callbacks() const
  {
    return callbacks_.size();
  }

  void unregister_callbacks()
  {
    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    callbacks_.clear();
  }
};

class MockPlayer : public FakePlayer
{
public:
  MOCK_METHOD(void, callback_func, (const std::string & keycode));
};

class KeyboardHandlerWindowsTest : public ::testing::Test
{
public:
  KeyboardHandlerWindowsTest()
  {
//    system_calls_stub = std::make_unique<NiceMock<MockSystemCalls>>();
  }

  ~KeyboardHandlerWindowsTest() override
  {
//    system_calls_stub.reset();
  }
};

TEST_F(KeyboardHandlerWindowsTest, KeyboardHandlerWithNullCallbackTest) {
  testing::MockFunction<void(const std::string & keycode)> mock_global_callback;
  EXPECT_CALL(mock_global_callback, Call(StrEq(""))).Times(0);

  MockKeyboardHandler keyboard_handler;
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_FALSE(keyboard_handler.add_key_press_callback(nullptr, "A"));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);

  EXPECT_FALSE(keyboard_handler.add_key_press_callback(nullptr, ""));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);

  EXPECT_FALSE(keyboard_handler.add_key_press_callback(mock_global_callback.AsStdFunction(), ""));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
}

#endif  // #ifdef _WIN32
