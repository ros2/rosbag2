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

using namespace std::chrono_literals;

namespace
{
int isatty_mock(int fd) {return 1;}

int tcgetattr_mock(int fd, struct termios * termios_p) {return 0;}

int tcsetattr_mock(int fd, int optional_actions, const struct termios * termios_p) {return 0;}
}  // namespace

ACTION_P(SetArg1FromCharPtr, param)
{
  strncpy(static_cast<char *>(arg1), param, arg2);  // NOLINT
}

class MockKeyboardHandler : public KeyboardHandlerUnixImpl
{
public:
  explicit MockKeyboardHandler(const readFunction & read_fn)
  : KeyboardHandlerUnixImpl(read_fn, isatty_mock, tcgetattr_mock, tcsetattr_mock) {}

  size_t get_number_of_registered_callbacks() const
  {
    return callbacks_.size();
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
  MockSystemCalls()
  {
    ON_CALL(*this, read(_, _, _)).WillByDefault(Return(0));
  }
  MOCK_METHOD(ssize_t, read, (int fd, void * buf_ptr, size_t n_bytes));
};

std::unique_ptr<NiceMock<MockSystemCalls>> system_calls_stub;

class KeyboardHandlerUnixTest : public ::testing::Test
{
public:
  KeyboardHandlerUnixTest()
  {
    system_calls_stub = std::make_unique<NiceMock<MockSystemCalls>>();
    read_fn_ = [](int fd, void * buf_ptr, size_t n_bytes) -> ssize_t {
        if (system_calls_stub) {
          return system_calls_stub->read(fd, buf_ptr, n_bytes);
        } else {
          std::cerr << "Call for non existing unique_ptr" << std::endl;
          return 0;
        }
      };
  }

  ~KeyboardHandlerUnixTest() override
  {
    system_calls_stub.reset();
  }

protected:
  KeyboardHandlerUnixImpl::readFunction read_fn_ = nullptr;
};

TEST_F(KeyboardHandlerUnixTest, keyboard_handler_with_null_callback) {
  EXPECT_CALL(*system_calls_stub, read(_, _, _)).WillRepeatedly(Return(0));

  testing::MockFunction<void(KeyboardHandler::KeyCode keycode)> mock_global_callback;
  EXPECT_CALL(mock_global_callback, Call(Eq(KeyboardHandler::KeyCode::UNKNOWN))).Times(0);

  MockKeyboardHandler keyboard_handler(read_fn_);
  ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_FALSE(
    keyboard_handler.add_key_press_callback(nullptr, KeyboardHandler::KeyCode::CAPITAL_A));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
}

TEST_F(KeyboardHandlerUnixTest, keyboard_handler_with_lambdas_in_callbacks) {
  auto recorder = FakeRecorder::create();
  std::shared_ptr<FakePlayer> player_shared_ptr(new FakePlayer());
  {
    MockKeyboardHandler keyboard_handler(read_fn_);
    const std::string terminal_seq =
      keyboard_handler.get_terminal_sequence(KeyboardHandler::KeyCode::CURSOR_UP);

    EXPECT_CALL(*system_calls_stub, read(_, _, _)).WillRepeatedly(
      testing::DoAll(
        SetArg1FromCharPtr(terminal_seq.c_str()), Return(terminal_seq.length())));

    ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);

    // Capture std::cout to verify at the end of the test that callbacks was correctly processed
    testing::internal::CaptureStdout();

    recorder->register_callbacks(keyboard_handler);
    ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);

    player_shared_ptr->register_callbacks(keyboard_handler);
    ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 2U);
  }
  // Check that callbacks was called with proper key code.
  std::string test_output = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(
    test_output.find("FakePlayer callback with key code = CURSOR_UP") != std::string::npos);
  EXPECT_TRUE(
    test_output.find("FakeRecorder callback with key code = CURSOR_UP") != std::string::npos);
}

TEST_F(KeyboardHandlerUnixTest, keyboard_handler_with_lambdas_in_callbacks_and_deleted_objects) {
  {
    MockKeyboardHandler keyboard_handler(read_fn_);
    const std::string terminal_seq =
      keyboard_handler.get_terminal_sequence(KeyboardHandler::KeyCode::CURSOR_UP);

    EXPECT_CALL(*system_calls_stub, read(_, _, _)).WillRepeatedly(
      testing::DoAll(
        SetArg1FromCharPtr(terminal_seq.c_str()), Return(terminal_seq.length())));

    ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
    {
      auto recorder = FakeRecorder::create();
      std::shared_ptr<FakePlayer> player_shared_ptr(new FakePlayer());

      // Capture std::cout to verify at the end of the test that callbacks for deleted objects
      // was correctly processed
      testing::internal::CaptureStdout();
      recorder->register_callbacks(keyboard_handler);
      ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);

      player_shared_ptr->register_callbacks(keyboard_handler);
      ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 2U);
    }
    // Yield CPU resources to give keyboard_handler a chance to process deleted callbacks.
    std::this_thread::sleep_for(1ms);
  }
  // Check that callbacks was called for deleted objects and processed properly
  std::string test_output = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(
    test_output.find("Object for assigned callback FakePlayer() was deleted") != std::string::npos);
  EXPECT_TRUE(
    test_output.find("Object for assigned callback FakeRecorder() was deleted") !=
    std::string::npos);
}

TEST_F(KeyboardHandlerUnixTest, keyboard_handler_with_global_callback) {
  testing::MockFunction<void(KeyboardHandler::KeyCode key_code)> mock_global_callback;
  EXPECT_CALL(
    mock_global_callback,
    Call(Eq(KeyboardHandler::KeyCode::CAPITAL_E))).Times(AtLeast(1));

  MockKeyboardHandler keyboard_handler(read_fn_);
  const std::string terminal_seq =
    keyboard_handler.get_terminal_sequence(KeyboardHandler::KeyCode::CAPITAL_E);

  EXPECT_CALL(*system_calls_stub, read(_, _, _)).WillRepeatedly(
    testing::DoAll(
      SetArg1FromCharPtr(terminal_seq.c_str()), Return(terminal_seq.length())));

  ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(
      mock_global_callback.AsStdFunction(), KeyboardHandler::KeyCode::CAPITAL_E));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);
}

TEST_F(KeyboardHandlerUnixTest, keyboard_handler_with_class_member_in_callback) {
  MockKeyboardHandler keyboard_handler(read_fn_);
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
