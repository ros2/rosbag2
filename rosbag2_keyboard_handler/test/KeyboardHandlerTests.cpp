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
#include <memory>
#include <algorithm>
#include "gmock/gmock.h"
#include "FakeRecorder.hpp"
#include "FakePlayer.hpp"
#include "rosbag2_keyboard_handler/KeyboardHandlerUnixImpl.hpp"

using ::testing::Return;
using ::testing::Eq;
using ::testing::StrEq;
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
  MOCK_METHOD(void, callback_func, (const std::string & keycode));
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
int isatty(int __fd) __THROW
{
  return 1;
}

KEYBOARD_HANDLER_PUBLIC
int tcgetattr(int __fd, struct termios * __termios_p) __THROW
{
  return 0;
}

KEYBOARD_HANDLER_PUBLIC
int tcsetattr(int __fd, int __optional_actions, const struct termios * __termios_p) __THROW
{
  return 0;
}

#ifdef __cplusplus
}
#endif

TEST_F(KeyboardHandlerUnixTest, KeyboardHandlerWithNullCallbackTest) {
  EXPECT_CALL(*system_calls_stub, read(_, _, _)).WillRepeatedly(Return(0));

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

TEST_F(KeyboardHandlerUnixTest, KeyboardHandlerWithLambdaAsCallbacksTest) {
  char buff[10] = {};
  strncpy(
    buff, KeyboardHandler::KEY_CODE_CURSOR_UP,
    std::min(sizeof(buff), sizeof(KeyboardHandler::KEY_CODE_CURSOR_UP)));
  EXPECT_CALL(*system_calls_stub, read(_, _, _))
  .WillRepeatedly(testing::DoAll(SetArg1FromCharPtr(buff), Return(strlen(buff))));

  auto recorder = FakeRecorder::create();
  std::shared_ptr<FakePlayer> player_shared_ptr(new FakePlayer());

  {
    MockKeyboardHandler keyboard_handler;
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
    test_output.find("FakePlayer callback with keycode = KEY_CODE_CURSOR_UP") != std::string::npos);
  EXPECT_TRUE(
    test_output.find(
      "FakeRecorder callback with keycode = KEY_CODE_CURSOR_UP") != std::string::npos);
}

TEST_F(KeyboardHandlerUnixTest, KeyboardHandlerWithLambdaAsCallbacksAndDeletedObjectsTest) {
  char buff[10] = {};
  strncpy(
    buff, KeyboardHandler::KEY_CODE_CURSOR_UP,
    std::min(sizeof(buff), sizeof(KeyboardHandler::KEY_CODE_CURSOR_UP)));

  EXPECT_CALL(*system_calls_stub, read(_, _, _))
  .WillRepeatedly(testing::DoAll(SetArg1FromCharPtr(buff), Return(strlen(buff))));

  {
    MockKeyboardHandler keyboard_handler;
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
  char buff[10] = "E";
  const std::string str_to_verify{buff};
  EXPECT_CALL(*system_calls_stub, read(_, _, _))
  .WillRepeatedly(testing::DoAll(SetArg1FromCharPtr(buff), Return(strlen(buff))));

  testing::MockFunction<void(const std::string & keycode)> mock_global_callback;
  EXPECT_CALL(mock_global_callback, Call(StrEq(str_to_verify))).Times(AtLeast(1));

  MockKeyboardHandler keyboard_handler;
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(mock_global_callback.AsStdFunction(), str_to_verify));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);
}

TEST_F(KeyboardHandlerUnixTest, KeyboardHandlerMockClassMemberTest) {
  char buff[10] = {};
  strncpy(
    buff, KeyboardHandler::KEY_CODE_CURSOR_DOWN,
    std::min(sizeof(buff), sizeof(KeyboardHandler::KEY_CODE_CURSOR_DOWN)));
  EXPECT_CALL(*system_calls_stub, read(_, _, _))
  .WillRepeatedly(testing::DoAll(SetArg1FromCharPtr(buff), Return(strlen(buff))));

  KeyboardHandler keyboard_handler;
  std::shared_ptr<NiceMock<MockPlayer>> mock_player_sptr(new NiceMock<MockPlayer>());
  const std::string keycode_cursor_down_str{KeyboardHandler::KEY_CODE_CURSOR_DOWN};
  const std::string keycode_cursor_up_str{KeyboardHandler::KEY_CODE_CURSOR_UP};
  EXPECT_CALL(*mock_player_sptr, callback_func(StrEq(keycode_cursor_down_str))).Times(AtLeast(1));
  EXPECT_CALL(*mock_player_sptr, callback_func(StrEq(keycode_cursor_up_str))).Times(0);

  auto callback = std::bind(&MockPlayer::callback_func, mock_player_sptr, std::placeholders::_1);
  EXPECT_TRUE(keyboard_handler.add_key_press_callback(callback, keycode_cursor_down_str));
  EXPECT_TRUE(keyboard_handler.add_key_press_callback(callback, keycode_cursor_up_str));
}
