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

#ifndef KEYBOARD_HANDLER__KEYBOARD_HANDLER_BASE_HPP_
#define KEYBOARD_HANDLER__KEYBOARD_HANDLER_BASE_HPP_

#include <functional>
#include <unordered_map>
#include <mutex>
#include <string>
#include "keyboard_handler/visibility_control.hpp"

class KeyboardHandlerBase
{
public:
  /// \brief Enum for possible keys press combinations which keyboard handler capable to handle.
  enum class KeyCode : uint32_t;

  /// \brief Type for callback functions
  using callback_t = std::function<void (KeyCode)>;

  /// \brief Adding callable object as a handler for specified key press combination.
  /// \param callback Callable which will be called when key_code will be recognized.
  /// \param key_code Value from enum which corresponds to some predefined key press combination.
  /// \return Return true if callback was successfully added to the keyboard handler, return
  /// false if callback is null_ptr or keyboard handler wasn't successfully initialized.
  KEYBOARD_HANDLER_PUBLIC
  bool add_key_press_callback(const callback_t & callback, KeyboardHandlerBase::KeyCode key_code);

protected:
  bool is_init_succeed_ = false;
  std::mutex callbacks_mutex_;
  std::unordered_multimap<KeyCode, callback_t> callbacks_;
};

enum class KeyboardHandlerBase::KeyCode: uint32_t
{
  UNKNOWN,
  NUMBER_1,
  NUMBER_2,
  NUMBER_3,
  NUMBER_4,
  NUMBER_5,
  NUMBER_6,
  NUMBER_7,
  NUMBER_8,
  NUMBER_9,
  NUMBER_0,
  MINUS,
  EQUAL_SIGN,
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L,
  M,
  N,
  O,
  P,
  Q,
  R,
  S,
  T,
  U,
  V,
  W,
  X,
  Y,
  Z,
  CAPITAL_A,
  CAPITAL_B,
  CAPITAL_C,
  CAPITAL_D,
  CAPITAL_E,
  CAPITAL_F,
  CAPITAL_G,
  CAPITAL_H,
  CAPITAL_I,
  CAPITAL_J,
  CAPITAL_K,
  CAPITAL_L,
  CAPITAL_M,
  CAPITAL_N,
  CAPITAL_O,
  CAPITAL_P,
  CAPITAL_Q,
  CAPITAL_R,
  CAPITAL_S,
  CAPITAL_T,
  CAPITAL_U,
  CAPITAL_V,
  CAPITAL_W,
  CAPITAL_X,
  CAPITAL_Y,
  CAPITAL_Z,
  CURSOR_UP,
  CURSOR_DOWN,
  CURSOR_LEFT,
  CURSOR_RIGHT,
  ESCAPE,
  SPACE,
  ENTER,
  BACK_SPACE,
  DELETE,
  END,
  PG_DOWN,
  PG_UP,
  HOME,
  INSERT,
  F1,
  F2,
  F3,
  F4,
  F5,
  F6,
  F7,
  F8,
  F9,
  F10,
  F11,
  F12,
  SHIFT_F1,
  SHIFT_F2,
  SHIFT_F3,
  SHIFT_F4,
  SHIFT_F5,
  SHIFT_F6,
  SHIFT_F7,
  SHIFT_F8,
  SHIFT_F9,
  SHIFT_F10,
  SHIFT_F11,
  SHIFT_F12,
};

/// \brief Data type for mapping KeyCode enum value to it's string representation.
struct KeyCodeToStrMap
{
  KeyboardHandlerBase::KeyCode inner_code;
  const char * str;
};

/// \brief Lookup table for mapping KeyCode enum value to it's string representation.
static const KeyCodeToStrMap ENUM_KEY_TO_STR_MAP[] {
  {KeyboardHandlerBase::KeyCode::UNKNOWN, "UNKNOWN"},
  {KeyboardHandlerBase::KeyCode::NUMBER_1, "NUMBER_1"},
  {KeyboardHandlerBase::KeyCode::NUMBER_2, "NUMBER_2"},
  {KeyboardHandlerBase::KeyCode::NUMBER_3, "NUMBER_3"},
  {KeyboardHandlerBase::KeyCode::NUMBER_4, "NUMBER_4"},
  {KeyboardHandlerBase::KeyCode::NUMBER_5, "NUMBER_5"},
  {KeyboardHandlerBase::KeyCode::NUMBER_6, "NUMBER_6"},
  {KeyboardHandlerBase::KeyCode::NUMBER_7, "NUMBER_7"},
  {KeyboardHandlerBase::KeyCode::NUMBER_8, "NUMBER_8"},
  {KeyboardHandlerBase::KeyCode::NUMBER_9, "NUMBER_9"},
  {KeyboardHandlerBase::KeyCode::NUMBER_0, "NUMBER_0"},
  {KeyboardHandlerBase::KeyCode::MINUS, "MINUS"},
  {KeyboardHandlerBase::KeyCode::EQUAL_SIGN, "EQUAL_SIGN"},
  {KeyboardHandlerBase::KeyCode::A, "a"},
  {KeyboardHandlerBase::KeyCode::B, "b"},
  {KeyboardHandlerBase::KeyCode::C, "c"},
  {KeyboardHandlerBase::KeyCode::D, "d"},
  {KeyboardHandlerBase::KeyCode::E, "e"},
  {KeyboardHandlerBase::KeyCode::F, "f"},
  {KeyboardHandlerBase::KeyCode::G, "g"},
  {KeyboardHandlerBase::KeyCode::H, "h"},
  {KeyboardHandlerBase::KeyCode::I, "i"},
  {KeyboardHandlerBase::KeyCode::J, "j"},
  {KeyboardHandlerBase::KeyCode::K, "k"},
  {KeyboardHandlerBase::KeyCode::L, "l"},
  {KeyboardHandlerBase::KeyCode::M, "m"},
  {KeyboardHandlerBase::KeyCode::N, "n"},
  {KeyboardHandlerBase::KeyCode::O, "o"},
  {KeyboardHandlerBase::KeyCode::P, "p"},
  {KeyboardHandlerBase::KeyCode::Q, "q"},
  {KeyboardHandlerBase::KeyCode::R, "r"},
  {KeyboardHandlerBase::KeyCode::S, "s"},
  {KeyboardHandlerBase::KeyCode::T, "t"},
  {KeyboardHandlerBase::KeyCode::U, "u"},
  {KeyboardHandlerBase::KeyCode::V, "v"},
  {KeyboardHandlerBase::KeyCode::W, "w"},
  {KeyboardHandlerBase::KeyCode::X, "x"},
  {KeyboardHandlerBase::KeyCode::Y, "y"},
  {KeyboardHandlerBase::KeyCode::Z, "z"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_A, "A"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_B, "B"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_C, "C"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_D, "D"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_E, "E"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_F, "F"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_G, "G"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_H, "H"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_I, "I"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_J, "J"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_K, "K"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_L, "L"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_M, "M"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_N, "N"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_O, "O"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_P, "P"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_Q, "Q"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_R, "R"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_S, "S"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_T, "T"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_U, "U"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_V, "V"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_W, "W"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_X, "X"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_Y, "Y"},
  {KeyboardHandlerBase::KeyCode::CAPITAL_Z, "Z"},
  {KeyboardHandlerBase::KeyCode::CURSOR_UP, "CURSOR_UP"},
  {KeyboardHandlerBase::KeyCode::CURSOR_DOWN, "CURSOR_DOWN"},
  {KeyboardHandlerBase::KeyCode::CURSOR_LEFT, "CURSOR_LEFT"},
  {KeyboardHandlerBase::KeyCode::CURSOR_RIGHT, "CURSOR_RIGHT"},
  {KeyboardHandlerBase::KeyCode::ESCAPE, "ESCAPE"},
  {KeyboardHandlerBase::KeyCode::SPACE, "SPACE"},
  {KeyboardHandlerBase::KeyCode::ENTER, "ENTER"},
  {KeyboardHandlerBase::KeyCode::BACK_SPACE, "BACK_SPACE"},
  {KeyboardHandlerBase::KeyCode::DELETE, "DELETE"},
  {KeyboardHandlerBase::KeyCode::END, "END"},
  {KeyboardHandlerBase::KeyCode::PG_DOWN, "PG_DOWN"},
  {KeyboardHandlerBase::KeyCode::PG_UP, "PG_UP"},
  {KeyboardHandlerBase::KeyCode::HOME, "HOME"},
  {KeyboardHandlerBase::KeyCode::INSERT, "INSERT"},
  {KeyboardHandlerBase::KeyCode::F1, "F1"},
  {KeyboardHandlerBase::KeyCode::F2, "F2"},
  {KeyboardHandlerBase::KeyCode::F3, "F3"},
  {KeyboardHandlerBase::KeyCode::F4, "F4"},
  {KeyboardHandlerBase::KeyCode::F5, "F5"},
  {KeyboardHandlerBase::KeyCode::F6, "F6"},
  {KeyboardHandlerBase::KeyCode::F7, "F7"},
  {KeyboardHandlerBase::KeyCode::F8, "F8"},
  {KeyboardHandlerBase::KeyCode::F9, "F9"},
  {KeyboardHandlerBase::KeyCode::F10, "F10"},
  {KeyboardHandlerBase::KeyCode::F11, "F11"},
  {KeyboardHandlerBase::KeyCode::F12, "F12"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F1, "SHIFT_F1"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F2, "SHIFT_F2"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F3, "SHIFT_F3"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F4, "SHIFT_F4"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F5, "SHIFT_F5"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F6, "SHIFT_F6"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F7, "SHIFT_F7"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F8, "SHIFT_F8"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F9, "SHIFT_F9"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F10, "SHIFT_F10"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F11, "SHIFT_F11"},
  {KeyboardHandlerBase::KeyCode::SHIFT_F12, "SHIFT_F12"},
};

/// \brief Translate KeyCode enum value to it's string representation.
/// \param key_code Value from enum which corresponds to some predefined key press combination.
/// \return String corresponding to the specified enum value in ENUM_KEY_TO_STR_MAP lookup table.
KEYBOARD_HANDLER_PUBLIC
std::string enum_key_code_to_str(KeyboardHandlerBase::KeyCode key_code);

#endif  // KEYBOARD_HANDLER__KEYBOARD_HANDLER_BASE_HPP_
