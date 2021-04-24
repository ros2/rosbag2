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

#ifndef _WIN32
#include <string>
#include <memory>
#include <algorithm>
#include "gmock/gmock.h"
#include "fake_recorder.hpp"
#include "fake_player.hpp"
#include "keyboard_handler/keyboard_handler_unix_impl.hpp"

using ::testing::Return;
using ::testing::Eq;
using ::testing::AtLeast;
using ::testing::_;
using ::testing::NiceMock;

ACTION_P(SetArg1FromCharPtr, param)
{
  strcpy(static_cast<char *>(arg1), param);  // NOLINT
}

class MockKeyboardHandler : public KeyboardHandlerUnixImpl
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

// Mock the public system calls APIs. read() function become the stub function.
class MockSystemCalls
{
public:
  MOCK_METHOD(ssize_t, read, (int __fd, void * __buf, size_t __nbytes));
};

std::unique_ptr<NiceMock<MockSystemCalls>> system_calls_stub;

class KeyboardHandlerUnixTest : public ::testing::Test
{
public:
  KeyboardHandlerUnixTest()
  {
    system_calls_stub = std::make_unique<NiceMock<MockSystemCalls>>();
  }

  ~KeyboardHandlerUnixTest() override
  {
    system_calls_stub.reset();
  }
};

#ifdef __cplusplus
extern "C"
{
#endif

KEYBOARD_HANDLER_PUBLIC
ssize_t read(int __fd, void * __buf, size_t __nbytes)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ssize_t ret = 0;
  if (system_calls_stub != nullptr) {
    ret = system_calls_stub->read(__fd, __buf, __nbytes);
  } else {
    std::cerr << "Call for non existing unique_ptr" << std::endl;
  }
  return ret;
}

KEYBOARD_HANDLER_PUBLIC
int isatty(int __fd)
{
  return 1;
}

KEYBOARD_HANDLER_PUBLIC
int tcgetattr(int __fd, struct termios * __termios_p)
{
  return 0;
}

KEYBOARD_HANDLER_PUBLIC
int tcsetattr(int __fd, int __optional_actions, const struct termios * __termios_p)
{
  return 0;
}

#ifdef __cplusplus
}
#endif

TEST_F(KeyboardHandlerUnixTest, KeyboardHandlerWithNullCallbackTest) {
  EXPECT_CALL(*system_calls_stub, read(_, _, _)).WillRepeatedly(Return(0));

  testing::MockFunction<void(KeyboardHandler::KeyCode keycode)> mock_global_callback;
  EXPECT_CALL(mock_global_callback, Call(Eq(KeyboardHandler::KeyCode::UNKNOWN))).Times(0);

  MockKeyboardHandler keyboard_handler;
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_FALSE(
    keyboard_handler.add_key_press_callback(
      nullptr,
      KeyboardHandler::KeyCode::CAPITAL_A));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
}

TEST_F(KeyboardHandlerUnixTest, KeyboardHandlerWithLambdaAsCallbacksTest) {
  auto recorder = FakeRecorder::create();
  std::shared_ptr<FakePlayer> player_shared_ptr(new FakePlayer());
  {
    MockKeyboardHandler keyboard_handler;
    const std::string terminal_seq =
      keyboard_handler.get_terminal_sequence(KeyboardHandler::KeyCode::CURSOR_UP);

    EXPECT_CALL(*system_calls_stub, read(_, _, _)).WillRepeatedly(
      testing::DoAll(
        SetArg1FromCharPtr(terminal_seq.c_str()), Return(terminal_seq.length())));

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

TEST_F(KeyboardHandlerUnixTest, KeyboardHandlerWithLambdaAsCallbacksAndDeletedObjectsTest) {
  {
    MockKeyboardHandler keyboard_handler;
    const std::string terminal_seq =
      keyboard_handler.get_terminal_sequence(KeyboardHandler::KeyCode::CURSOR_UP);

    EXPECT_CALL(*system_calls_stub, read(_, _, _)).WillRepeatedly(
      testing::DoAll(
        SetArg1FromCharPtr(terminal_seq.c_str()), Return(terminal_seq.length())));

    EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
    {
      auto recorder = FakeRecorder::create();
      std::shared_ptr<FakePlayer> player_shared_ptr(new FakePlayer());

      // Capture std::cout to verify at the end of the test that callbacks for deleted objects
      // was correctly processed
      testing::internal::CaptureStdout();
      recorder->register_callbacks(keyboard_handler);
      EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);

      player_shared_ptr->register_callbacks(keyboard_handler);
      EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 2U);
    }
  }
  // Check that callbacks was called for deleted objects and processed properly
  std::string test_output = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(
    test_output.find("Object for assigned callback FakePlayer() was deleted") != std::string::npos);
  EXPECT_TRUE(
    test_output.find("Object for assigned callback FakeRecorder() was deleted") !=
    std::string::npos);
}

TEST_F(KeyboardHandlerUnixTest, KeyboardHandlerGlobalCallbackTest) {
  testing::MockFunction<void(KeyboardHandler::KeyCode key_code)> mock_global_callback;
  EXPECT_CALL(
    mock_global_callback,
    Call(Eq(KeyboardHandler::KeyCode::CAPITAL_E))).Times(AtLeast(1));

  MockKeyboardHandler keyboard_handler;
  const std::string terminal_seq =
    keyboard_handler.get_terminal_sequence(KeyboardHandler::KeyCode::CAPITAL_E);

  EXPECT_CALL(*system_calls_stub, read(_, _, _)).WillRepeatedly(
    testing::DoAll(
      SetArg1FromCharPtr(terminal_seq.c_str()), Return(terminal_seq.length())));

  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(
      mock_global_callback.AsStdFunction(), KeyboardHandler::KeyCode::CAPITAL_E));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);
}

TEST_F(KeyboardHandlerUnixTest, KeyboardHandlerMockClassMemberTest) {
  KeyboardHandler keyboard_handler;
  const std::string terminal_seq =
    keyboard_handler.get_terminal_sequence(KeyboardHandler::KeyCode::CURSOR_DOWN);

  EXPECT_CALL(*system_calls_stub, read(_, _, _)).WillRepeatedly(
    testing::DoAll(
      SetArg1FromCharPtr(terminal_seq.c_str()), Return(terminal_seq.length())));

  std::shared_ptr<NiceMock<MockPlayer>> mock_player_sptr(new NiceMock<MockPlayer>());
  EXPECT_CALL(
    *mock_player_sptr,
    callback_func(Eq(KeyboardHandler::KeyCode::CURSOR_DOWN))).Times(AtLeast(1));
  EXPECT_CALL(*mock_player_sptr, callback_func(Eq(KeyboardHandler::KeyCode::CURSOR_UP))).Times(0);

  auto callback = std::bind(&MockPlayer::callback_func, mock_player_sptr, std::placeholders::_1);
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::CURSOR_UP));
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::CURSOR_DOWN));
}
#endif  // #ifndef _WIN32
