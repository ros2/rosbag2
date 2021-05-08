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
#include <utility>
#include <condition_variable>
#include "gmock/gmock.h"
#include "fake_recorder.hpp"
#include "fake_player.hpp"
#include "keyboard_handler/keyboard_handler_unix_impl.hpp"

using ::testing::Return;
using ::testing::Eq;
using ::testing::AtLeast;
using ::testing::_;
using ::testing::NiceMock;

namespace
{
int isatty_mock(int fd) {return 1;}

int tcgetattr_mock(int fd, struct termios * termios_p) {return 0;}

int tcsetattr_mock(int fd, int optional_actions, const struct termios * termios_p) {return 0;}
}  // namespace

// Mock the public system calls APIs. read() function become the stub function.
class MockSystemCalls
{
public:
  ssize_t read(int fd, void * buff_ptr, size_t n_bytes)
  {
    std::unique_lock<std::mutex> lk(read_fn_mutex_);
    if (wait_on_read_) {
      cv_read_.wait(lk, [this]() {return unblock_read_;});
      unblock_read_ = false;
    }
    strncpy(static_cast<char *>(buff_ptr), read_returning_str_value_.c_str(), n_bytes);
    return read_returning_str_value_.length();
  }

  void read_will_return_once(const std::string & str)
  {
    {
      std::lock_guard<std::mutex> lk(read_fn_mutex_);
      read_returning_str_value_ = str;
      wait_on_read_ = true;
      unblock_read_ = true;
    }
    cv_read_.notify_all();
  }

  void unblock_read()
  {
    {
      std::lock_guard<std::mutex> lk(read_fn_mutex_);
      unblock_read_ = true;
      wait_on_read_ = false;
    }
    cv_read_.notify_all();
  }

  void read_will_repeatedly_return(const std::string & str)
  {
    {
      std::lock_guard<std::mutex> lk(read_fn_mutex_);
      read_returning_str_value_ = str;
      unblock_read_ = true;
      wait_on_read_ = false;
    }
    cv_read_.notify_all();
  }

private:
  std::mutex read_fn_mutex_;
  std::condition_variable cv_read_;
  // By default read will block and wait
  bool wait_on_read_{true};
  bool unblock_read_{false};
  std::string read_returning_str_value_{};
};

std::shared_ptr<MockSystemCalls> g_system_calls_stub;

class MockKeyboardHandler : public KeyboardHandlerUnixImpl
{
public:
  explicit MockKeyboardHandler(
    const readFunction & read_fn,
    std::weak_ptr<MockSystemCalls> system_calls_stub = g_system_calls_stub)
  : KeyboardHandlerUnixImpl(read_fn, isatty_mock, tcgetattr_mock, tcsetattr_mock),
    system_calls_stub_(std::move(system_calls_stub)) {}

  ~MockKeyboardHandler() override
  {
    auto sys_calls_stub = system_calls_stub_.lock();
    if (sys_calls_stub) {
      // unlock read to let inner worker thread to finish
      sys_calls_stub->unblock_read();
    }
  }
  size_t get_number_of_registered_callbacks() const
  {
    return callbacks_.size();
  }

private:
  std::weak_ptr<MockSystemCalls> system_calls_stub_;
};

class MockPlayer : public FakePlayer
{
public:
  MOCK_METHOD(void, callback_func, (KeyboardHandler::KeyCode key_code));
};

class KeyboardHandlerUnixTest : public ::testing::Test
{
public:
  KeyboardHandlerUnixTest()
  {
    g_system_calls_stub = std::make_shared<MockSystemCalls>();
    read_fn_ = [](int fd, void * buf_ptr, size_t n_bytes) -> ssize_t {
        if (g_system_calls_stub) {
          return g_system_calls_stub->read(fd, buf_ptr, n_bytes);
        } else {
          std::cerr << "Call for non existing unique_ptr" << std::endl;
          return 0;
        }
      };
  }

  ~KeyboardHandlerUnixTest() override
  {
    g_system_calls_stub.reset();
  }

protected:
  KeyboardHandlerUnixImpl::readFunction read_fn_ = nullptr;
};

TEST_F(KeyboardHandlerUnixTest, keyboard_handler_with_null_callback) {
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
    ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);

    // Capture std::cout to verify at the end of the test that callbacks was correctly processed
    testing::internal::CaptureStdout();

    recorder->register_callbacks(keyboard_handler);
    ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);

    player_shared_ptr->register_callbacks(keyboard_handler);
    ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 2U);
    g_system_calls_stub->read_will_return_once(terminal_seq);
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
    g_system_calls_stub->read_will_return_once(terminal_seq);
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

  ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(
      mock_global_callback.AsStdFunction(), KeyboardHandler::KeyCode::CAPITAL_E));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);
  g_system_calls_stub->read_will_return_once(terminal_seq);
}

TEST_F(KeyboardHandlerUnixTest, keyboard_handler_with_class_member_in_callback) {
  MockKeyboardHandler keyboard_handler(read_fn_);
  const std::string terminal_seq =
    keyboard_handler.get_terminal_sequence(KeyboardHandler::KeyCode::CURSOR_DOWN);

  std::shared_ptr<NiceMock<MockPlayer>> mock_player_shared_ptr(new NiceMock<MockPlayer>());
  EXPECT_CALL(
    *mock_player_shared_ptr,
    callback_func(Eq(KeyboardHandler::KeyCode::CURSOR_DOWN))).Times(AtLeast(1));
  EXPECT_CALL(
    *mock_player_shared_ptr,
    callback_func(Eq(KeyboardHandler::KeyCode::CURSOR_UP))).Times(0);

  auto callback = std::bind(
    &MockPlayer::callback_func, mock_player_shared_ptr,
    std::placeholders::_1);
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::CURSOR_UP));
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::CURSOR_DOWN));
  g_system_calls_stub->read_will_return_once(terminal_seq);
}
#endif  // #ifndef _WIN32
