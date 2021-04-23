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
  MOCK_METHOD(void, callback_func, (KeyboardHandler::KeyCode key_code));
};

// Mock the public system calls APIs.
class MockSystemCalls
{
public:
  using WinKeyCode = KeyboardHandlerWindowsImpl::WinKeyCode;
  WinKeyCode win_key_code{WinKeyCode::NOT_A_KEY, WinKeyCode::NOT_A_KEY};

  int getch_win_code()
  {
    static size_t counter_ = 0;
    int ret_value = (counter_ % 2) ? win_key_code.second : win_key_code.first;
    counter_++;
    return ret_value;
  }
  MOCK_METHOD(int, _kbhit, ());
  MOCK_METHOD(int, _isatty, (int __fd));
  MOCK_METHOD(int, _getch, ());
};

std::unique_ptr<NiceMock<MockSystemCalls>> system_calls_stub;

class KeyboardHandlerWindowsTest : public ::testing::Test
{
public:
  KeyboardHandlerWindowsTest()
  {
    system_calls_stub = std::make_unique<NiceMock<MockSystemCalls>>();
  }

  ~KeyboardHandlerWindowsTest() override
  {
    system_calls_stub.reset();
  }
};

#ifdef __cplusplus
extern "C"
{
#endif

KEYBOARD_HANDLER_PUBLIC
int __cdecl _isatty(int _FileHandle)
{
  return 1;
}

KEYBOARD_HANDLER_PUBLIC
int __cdecl _kbhit(void)
{
  ssize_t ret = 0;
  if (system_calls_stub != nullptr) {
    ret = system_calls_stub->_kbhit();
  } else {
    std::cerr << "Call to '_kbhit()' for non existing unique_ptr" << std::endl;
  }
  return ret;
}

KEYBOARD_HANDLER_PUBLIC
int __cdecl _getch(void)
{
  ssize_t ret = 0;
  if (system_calls_stub != nullptr) {
    ret = system_calls_stub->_getch();
  } else {
    std::cerr << "Call to '_getch()' for non existing unique_ptr" << std::endl;
  }
  return ret;
}
#ifdef __cplusplus
}
#endif

TEST_F(KeyboardHandlerWindowsTest, KeyboardHandlerWithNullCallbackTest) {
  testing::MockFunction<void(KeyboardHandler::KeyCode keycode)> mock_global_callback;
  EXPECT_CALL(mock_global_callback, Call(Eq(KeyboardHandler::KeyCode::UNKNOWN))).Times(0);

  MockKeyboardHandler keyboard_handler;
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_FALSE(
    keyboard_handler.add_key_press_callback(nullptr, KeyboardHandler::KeyCode::CAPITAL_A));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
}

TEST_F(KeyboardHandlerWindowsTest, KeyboardHandlerWithLambdaAsCallbacksTest) {
  EXPECT_CALL(*system_calls_stub, _kbhit()).WillRepeatedly(Return(1));
  ON_CALL(*system_calls_stub, _getch()).WillByDefault(Return(-1));

  auto recorder = FakeRecorder::create();
  std::shared_ptr<FakePlayer> player_shared_ptr(new FakePlayer());

  {
    MockKeyboardHandler keyboard_handler;
    KeyboardHandlerWindowsImpl::WinKeyCode key_code =
      keyboard_handler.enum_key_code_to_win_code(KeyboardHandler::KeyCode::CURSOR_UP);

    system_calls_stub->win_key_code = key_code;

    EXPECT_CALL(*system_calls_stub, _getch())
    .WillRepeatedly(::testing::Invoke(system_calls_stub.get(), &MockSystemCalls::getch_win_code));

    EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);

    // Capture std::cout to verify at the end of the test that callbacks was correctly processed
    testing::internal::CaptureStdout();

    recorder->register_callbacks(keyboard_handler);
    EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);

    player_shared_ptr->register_callbacks(keyboard_handler);
    EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 2U);
  }
  // Check that callbacks was called with proper key code.
  std::string test_output = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(
    test_output.find("FakePlayer callback with key code = CURSOR_UP") != std::string::npos);
  EXPECT_TRUE(
    test_output.find("FakeRecorder callback with key code = CURSOR_UP") != std::string::npos);
}

TEST_F(KeyboardHandlerWindowsTest, KeyboardHandlerGlobalCallbackTest) {
  testing::MockFunction<void(KeyboardHandler::KeyCode key_code)> mock_global_callback;
  EXPECT_CALL(
    mock_global_callback,
    Call(Eq(KeyboardHandler::KeyCode::CAPITAL_E))).Times(AtLeast(1));

  EXPECT_CALL(*system_calls_stub, _kbhit()).WillRepeatedly(Return(1));
  EXPECT_CALL(*system_calls_stub, _getch()).WillRepeatedly(Return('E'));

  MockKeyboardHandler keyboard_handler;
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(
      mock_global_callback.AsStdFunction(),
      KeyboardHandler::KeyCode::CAPITAL_E));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);
}

TEST_F(KeyboardHandlerWindowsTest, KeyboardHandlerMockClassMemberTest) {
  EXPECT_CALL(*system_calls_stub, _kbhit()).WillRepeatedly(Return(1));
  EXPECT_CALL(*system_calls_stub, _getch()).WillRepeatedly(Return('Z'));

  KeyboardHandler keyboard_handler;
  std::shared_ptr<NiceMock<MockPlayer>> mock_player_sptr(new NiceMock<MockPlayer>());
  EXPECT_CALL(
    *mock_player_sptr,
    callback_func(Eq(KeyboardHandler::KeyCode::CAPITAL_Z))).Times(AtLeast(1));
  EXPECT_CALL(*mock_player_sptr, callback_func(Eq(KeyboardHandler::KeyCode::UNKNOWN))).Times(0);

  auto callback = std::bind(&MockPlayer::callback_func, mock_player_sptr, std::placeholders::_1);
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::CAPITAL_Z));
  EXPECT_TRUE(keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::UNKNOWN));
}

#endif  // #ifdef _WIN32
