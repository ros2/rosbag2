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
#include "gmock/gmock.h"
#include "fake_recorder.hpp"
#include "fake_player.hpp"
#include "keyboard_handler/keyboard_handler_windows_impl.hpp"

using ::testing::Return;
using ::testing::Eq;
using ::testing::AtLeast;
using ::testing::_;
using ::testing::NiceMock;

using namespace std::chrono_literals;

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
    ON_CALL(*this, kbhit).WillByDefault(Return(0));
    ON_CALL(*this, getch).WillByDefault(Return(-1));
  }

  int getch_win_code()
  {
    static size_t counter_ = 0;
    std::lock_guard<std::mutex> lk(win_key_code_mutex);
    int ret_value = (counter_ % 2) ? win_key_code_.second : win_key_code_.first;
    counter_++;
    return ret_value;
  }
  MOCK_METHOD(int, kbhit, ());
  MOCK_METHOD(int, getch, ());

private:
  std::mutex win_key_code_mutex;
  WinKeyCode win_key_code_{WinKeyCode::NOT_A_KEY, WinKeyCode::NOT_A_KEY};
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

namespace
{
int isatty_mock(int file_handle)
{
  return 1;
}

int kbhit_mock(void)
{
  int ret = 0;
  if (system_calls_stub != nullptr) {
    ret = system_calls_stub->kbhit();
  } else {
    std::cerr << "Call to '_kbhit()' for non existing unique_ptr" << std::endl;
  }
  return ret;
}

int getch_mock(void)
{
  int ret = -1;
  if (system_calls_stub != nullptr) {
    ret = system_calls_stub->getch();
  } else {
    std::cerr << "Call to '_getch()' for non existing unique_ptr" << std::endl;
  }
  return ret;
}
}  // namespace

class MockKeyboardHandler : public KeyboardHandlerWindowsImpl
{
public:
  MockKeyboardHandler()
  : KeyboardHandlerWindowsImpl(isatty_mock, kbhit_mock, getch_mock) {}

  size_t get_number_of_registered_callbacks() const
  {
    return callbacks_.size();
  }
};

TEST_F(KeyboardHandlerWindowsTest, keyboard_handler_with_null_callback) {
  testing::MockFunction<void(KeyboardHandler::KeyCode keycode)> mock_global_callback;
  EXPECT_CALL(mock_global_callback, Call(Eq(KeyboardHandler::KeyCode::UNKNOWN))).Times(0);

  MockKeyboardHandler keyboard_handler;
  ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_FALSE(
    keyboard_handler.add_key_press_callback(nullptr, KeyboardHandler::KeyCode::CAPITAL_A));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
}

TEST_F(KeyboardHandlerWindowsTest, keyboard_handler_with_lambdas_in_callbacks) {
  EXPECT_CALL(*system_calls_stub, kbhit()).WillRepeatedly(Return(1));

  auto recorder = FakeRecorder::create();
  std::shared_ptr<FakePlayer> player_shared_ptr(new FakePlayer());
  {
    MockKeyboardHandler keyboard_handler;
    system_calls_stub->set_returning_getch_code(
      keyboard_handler.enum_key_code_to_win_code(KeyboardHandler::KeyCode::CURSOR_UP));
    EXPECT_CALL(*system_calls_stub, getch())
    .WillRepeatedly(::testing::Invoke(system_calls_stub.get(), &MockSystemCalls::getch_win_code));

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

TEST_F(KeyboardHandlerWindowsTest, keyboard_handler_with_lambdas_in_callbacks_and_deleted_objects) {
  EXPECT_CALL(*system_calls_stub, kbhit()).WillRepeatedly(Return(1));
  {
    MockKeyboardHandler keyboard_handler;
    system_calls_stub->set_returning_getch_code(
      keyboard_handler.enum_key_code_to_win_code(KeyboardHandler::KeyCode::CURSOR_UP));
    EXPECT_CALL(*system_calls_stub, getch())
    .WillRepeatedly(::testing::Invoke(system_calls_stub.get(), &MockSystemCalls::getch_win_code));

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

TEST_F(KeyboardHandlerWindowsTest, keyboard_handler_with_global_callback) {
  testing::MockFunction<void(KeyboardHandler::KeyCode key_code)> mock_global_callback;
  EXPECT_CALL(
    mock_global_callback,
    Call(Eq(KeyboardHandler::KeyCode::CAPITAL_E))).Times(AtLeast(1));

  EXPECT_CALL(*system_calls_stub, kbhit()).WillRepeatedly(Return(1));
  EXPECT_CALL(*system_calls_stub, getch()).WillRepeatedly(Return('E'));

  MockKeyboardHandler keyboard_handler;
  ASSERT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 0U);
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(
      mock_global_callback.AsStdFunction(),
      KeyboardHandler::KeyCode::CAPITAL_E));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 1U);
}

TEST_F(KeyboardHandlerWindowsTest, keyboard_handler_with_class_member_in_callback) {
  EXPECT_CALL(*system_calls_stub, kbhit()).WillRepeatedly(Return(1));
  EXPECT_CALL(*system_calls_stub, getch()).WillRepeatedly(Return('Z'));

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
  EXPECT_TRUE(
    keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::CAPITAL_Z));
  EXPECT_TRUE(keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::UNKNOWN));
  EXPECT_EQ(keyboard_handler.get_number_of_registered_callbacks(), 2U);
}

#endif  // #ifdef _WIN32
