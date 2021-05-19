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
#include <mutex>
#include <condition_variable>
#include <utility>
#include "gmock/gmock.h"
#include "fake_recorder.hpp"
#include "fake_player.hpp"
#include "keyboard_handler/keyboard_handler_windows_impl.hpp"

using ::testing::Return;
using ::testing::Eq;
using ::testing::AtLeast;
using ::testing::_;
using ::testing::NiceMock;

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

  void set_returning_getch_code(WinKeyCode key_code)
  {
    std::lock_guard<std::mutex> lk(win_key_code_mutex);
    win_key_code_ = key_code;
  }

  MockSystemCalls()
  {
    ON_CALL(*this, getch).WillByDefault(Return(-1));
  }

  int kbhit()
  {
    std::unique_lock<std::mutex> lk(kbhit_mutex_);
    if (wait_on_kbhit_) {
      cv_kbhit.wait(lk, [this]() {return unblock_kbhit_;});
      unblock_kbhit_ = false;
    }
    return kbhit_return_value_;
  }

  void kbhit_will_return_once(int ret_value)
  {
    {
      std::lock_guard<std::mutex> lk(kbhit_mutex_);
      kbhit_return_value_ = ret_value;
      wait_on_kbhit_ = true;
      unblock_kbhit_ = true;
    }
    cv_kbhit.notify_all();
  }

  void kbhit_will_repeatedly_return(int ret_value)
  {
    {
      std::lock_guard<std::mutex> lk(kbhit_mutex_);
      kbhit_return_value_ = ret_value;
      wait_on_kbhit_ = false;
      unblock_kbhit_ = true;
    }
    cv_kbhit.notify_all();
  }

  void unblock_kbhit()
  {
    {
      std::lock_guard<std::mutex> lk(kbhit_mutex_);
      wait_on_kbhit_ = false;
      unblock_kbhit_ = true;
    }
    cv_kbhit.notify_all();
  }

  int getch_win_code()
  {
    static size_t counter_ = 0;
    std::lock_guard<std::mutex> lk(win_key_code_mutex);
    int ret_value = (counter_ % 2) ? win_key_code_.second : win_key_code_.first;
    counter_++;
    return ret_value;
  }
  MOCK_METHOD(int, getch, ());

private:
  std::mutex kbhit_mutex_;
  std::condition_variable cv_kbhit;
  bool wait_on_kbhit_ = true;
  bool unblock_kbhit_ = false;
  int kbhit_return_value_ = 0;
  std::mutex win_key_code_mutex;
  WinKeyCode win_key_code_{WinKeyCode::NOT_A_KEY, WinKeyCode::NOT_A_KEY};
};

std::shared_ptr<NiceMock<MockSystemCalls>> g_system_calls_stub;

class KeyboardHandlerWindowsTest : public ::testing::Test
{
public:
  KeyboardHandlerWindowsTest()
  {
    g_system_calls_stub = std::make_shared<NiceMock<MockSystemCalls>>();
  }

  ~KeyboardHandlerWindowsTest() override
  {
    g_system_calls_stub.reset();
  }
};

namespace
{
int isatty_mock(int file_handle)
{
  return 1;
}

int kbhit_mock(void)
{
  int ret = 0;
  if (g_system_calls_stub != nullptr) {
    ret = g_system_calls_stub->kbhit();
  } else {
    std::cerr << "Call to '_kbhit()' for non existing unique_ptr" << std::endl;
  }
  return ret;
}

int getch_mock(void)
{
  int ret = -1;
  if (g_system_calls_stub != nullptr) {
    ret = g_system_calls_stub->getch();
  } else {
    std::cerr << "Call to '_getch()' for non existing unique_ptr" << std::endl;
  }
  return ret;
}
}  // namespace

class MockKeyboardHandler : public KeyboardHandlerWindowsImpl
{
public:
  explicit MockKeyboardHandler(
    std::weak_ptr<NiceMock<MockSystemCalls>> system_calls_stub = g_system_calls_stub)
  : KeyboardHandlerWindowsImpl(isatty_mock, kbhit_mock, getch_mock),
    system_calls_stub_(std::move(system_calls_stub)) {}

  ~MockKeyboardHandler() override
  {
    auto system_calls_stub = system_calls_stub_.lock();
    if (system_calls_stub) {
      // unlock kbhit to let inner worker thread to finish
      system_calls_stub->unblock_kbhit();
    }
  }

  size_t get_number_of_registered_callbacks() const
  {
    return callbacks_.size();
  }

private:
  std::weak_ptr<NiceMock<MockSystemCalls>> system_calls_stub_;
};

TEST_F(KeyboardHandlerWindowsTest, nullptr_as_callback) {
  MockKeyboardHandler keyboard_handler;
  ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_EQ(
    KeyboardHandler::invalid_handle,
    keyboard_handler.add_key_press_callback(nullptr, KeyboardHandler::KeyCode::CAPITAL_A));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
}

TEST_F(KeyboardHandlerWindowsTest, unregister_callback) {
  EXPECT_CALL(*g_system_calls_stub, getch()).WillRepeatedly(Return('E'));
  MockKeyboardHandler keyboard_handler;
  auto lambda_as_callback = [](KeyboardHandler::KeyCode key_code) {
      ASSERT_FALSE(true) << "This code should not be called \n";
    };
  auto callback_handle = keyboard_handler.add_key_press_callback(
    lambda_as_callback, KeyboardHandler::KeyCode::CAPITAL_E);
  EXPECT_NE(callback_handle, KeyboardHandler::invalid_handle);
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);

  keyboard_handler.delete_key_press_callback(callback_handle);
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);

  // Try to delete callback one more time to make sure that it will be handled correctly
  keyboard_handler.delete_key_press_callback(callback_handle);
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  g_system_calls_stub->kbhit_will_repeatedly_return(1);
}

TEST_F(KeyboardHandlerWindowsTest, weak_ptr_in_callbacks) {
  auto recorder = FakeRecorder::create();
  std::shared_ptr<FakePlayer> player_shared_ptr(new FakePlayer());
  {
    MockKeyboardHandler keyboard_handler;
    g_system_calls_stub->set_returning_getch_code(
      keyboard_handler.enum_key_code_to_win_code(KeyboardHandler::KeyCode::CURSOR_UP));
    EXPECT_CALL(*g_system_calls_stub, getch())
    .WillRepeatedly(::testing::Invoke(g_system_calls_stub.get(), &MockSystemCalls::getch_win_code));

    ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);

    // Capture std::cout to verify at the end of the test that callbacks was correctly processed
    testing::internal::CaptureStdout();

    recorder->register_callbacks(keyboard_handler);
    ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);

    player_shared_ptr->register_callbacks(keyboard_handler);
    ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 2U);
    g_system_calls_stub->kbhit_will_return_once(1);
  }
  // Check that callbacks was called with proper key code.
  std::string test_output = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(
    test_output.find("FakePlayer callback with key code = CURSOR_UP") != std::string::npos);
  EXPECT_TRUE(
    test_output.find("FakeRecorder callback with key code = CURSOR_UP") != std::string::npos);
}

TEST_F(KeyboardHandlerWindowsTest, weak_ptr_in_callbacks_and_deleted_objects) {
  {
    MockKeyboardHandler keyboard_handler;
    g_system_calls_stub->set_returning_getch_code(
      keyboard_handler.enum_key_code_to_win_code(KeyboardHandler::KeyCode::CURSOR_UP));
    EXPECT_CALL(*g_system_calls_stub, getch())
    .WillRepeatedly(::testing::Invoke(g_system_calls_stub.get(), &MockSystemCalls::getch_win_code));

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
    g_system_calls_stub->kbhit_will_return_once(1);
  }
  // Check that callbacks was called for deleted objects and processed properly
  std::string test_output = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(
    test_output.find("Object for assigned callback FakePlayer() was deleted") != std::string::npos);
  EXPECT_TRUE(
    test_output.find("Object for assigned callback FakeRecorder() was deleted") !=
    std::string::npos);
}

TEST_F(KeyboardHandlerWindowsTest, global_function_as_callback) {
  testing::MockFunction<void(KeyboardHandler::KeyCode key_code)> mock_global_callback;
  EXPECT_CALL(
    mock_global_callback,
    Call(Eq(KeyboardHandler::KeyCode::CAPITAL_E))).Times(AtLeast(1));

  EXPECT_CALL(*g_system_calls_stub, getch()).WillRepeatedly(Return('E'));
  g_system_calls_stub->kbhit_will_repeatedly_return(1);

  MockKeyboardHandler keyboard_handler;
  ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_NE(
    KeyboardHandler::invalid_handle,
    keyboard_handler.add_key_press_callback(
      mock_global_callback.AsStdFunction(), KeyboardHandler::KeyCode::CAPITAL_E));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);
}

TEST_F(KeyboardHandlerWindowsTest, class_member_as_callback) {
  EXPECT_CALL(*g_system_calls_stub, getch()).WillRepeatedly(Return('Z'));
  g_system_calls_stub->kbhit_will_repeatedly_return(1);

  MockKeyboardHandler keyboard_handler;
  std::shared_ptr<NiceMock<MockPlayer>> mock_player_shared_ptr(new NiceMock<MockPlayer>());
  EXPECT_CALL(
    *mock_player_shared_ptr,
    callback_func(Eq(KeyboardHandler::KeyCode::CAPITAL_Z))).Times(AtLeast(1));
  EXPECT_CALL(
    *mock_player_shared_ptr,
    callback_func(Eq(KeyboardHandler::KeyCode::UNKNOWN))).Times(0);

  auto callback = std::bind(
    &MockPlayer::callback_func, mock_player_shared_ptr,
    std::placeholders::_1);
  EXPECT_NE(
    KeyboardHandler::invalid_handle,
    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::CAPITAL_Z));
  EXPECT_NE(
    KeyboardHandler::invalid_handle,
    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::UNKNOWN));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 2U);
}

#endif  // #ifdef _WIN32
