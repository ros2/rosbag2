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
#include "keyboard_handler/keyboard_handler_unix_impl.hpp"

/// Note that key code sequences translated by the terminal could be differ for different terminal
/// emulators. Please refer to the
/// https://ftp.metu.edu.tr/pub/mirrors/ftp.x.org/pub/X11R6.8.0/PDF/ctlseqs.pdf. Also according
/// to the https://man7.org/linux/man-pages/man5/terminfo.5.html application could reassign some
/// some key code sequences.

//* *INDENT-OFF* */
namespace xterm_seq {
static constexpr char CURSOR_UP[]              = {27, 91, 65, '\0'};
static constexpr char CURSOR_DOWN[]            = {27, 91, 66, '\0'};
static constexpr char CURSOR_ONE_STEP_RIGHT[]  = {27, 91, 67, '\0'};
static constexpr char CURSOR_ONE_STEP_LEFT[]   = {27, 91, 68, '\0'};
static constexpr char SPACE[]                  = {32, '\0'};
static constexpr char ESC[]                    = {27, '\0'};
static constexpr char ENTER[]                  = {10, '\0'};
static constexpr char BACK_SPACE[]             = {127, '\0'};

static constexpr char DELETE[]                 = {27, 91, 51, 126, '\0'};
static constexpr char END[]                    = {27, 91, 70, '\0'};
static constexpr char PG_DOWN[]                = {27, 91, 54, 126, '\0'};
static constexpr char PG_UP[]                  = {27, 91, 53, 126, '\0'};
static constexpr char HOME[]                   = {27, 91, 72, '\0'};
static constexpr char INSERT[]                 = {27, 91, 50, 126, '\0'};

static constexpr char F1[]  = {27, 79, 80, '\0'};  // == {27, 79, 'P', '\0'};
static constexpr char F2[]  = {27, 79, 81, '\0'};
static constexpr char F3[]  = {27, 79, 82, '\0'};
static constexpr char F4[]  = {27, 79, 83, '\0'};
static constexpr char F5[]  = {27, 91, 49, 53, 126, '\0'};
static constexpr char F6[]  = {27, 91, 49, 55, 126, '\0'};
static constexpr char F7[]  = {27, 91, 49, 56, 126, '\0'};
static constexpr char F8[]  = {27, 91, 49, 57, 126, '\0'};
static constexpr char F9[]  = {27, 91, 50, 48, 126, '\0'};
static constexpr char F10[] = {27, 91, 50, 49, 126, '\0'};
static constexpr char F11[] = {27, 91, 50, 51, 126, '\0'};
static constexpr char F12[] = {27, 91, 50, 52, 126, '\0'};

static constexpr char SHIFT_F1[]  = {27, 91, 49, 59, 50, 80, '\0'};
static constexpr char SHIFT_F2[]  = {27, 91, 49, 59, 50, 81, '\0'};
static constexpr char SHIFT_F3[]  = {27, 91, 49, 59, 50, 82, '\0'};
static constexpr char SHIFT_F4[]  = {27, 91, 49, 59, 50, 83, '\0'};
static constexpr char SHIFT_F5[]  = {27, 91, 49, 53, 59, 50, 126, '\0'};
// Example: The same value for SHIFT_F5[] when numbers replaced by chars
// static constexpr char SHIFT_F5[] = {27, 91, '1', '5', ';', '2', '~', '\0'};
static constexpr char SHIFT_F6[]  = {27, 91, 49, 55, 59, 50, 126, '\0'};
static constexpr char SHIFT_F7[]  = {27, 91, 49, 56, 59, 50, 126, '\0'};
static constexpr char SHIFT_F8[]  = {27, 91, 49, 57, 59, 50, 126, '\0'};
static constexpr char SHIFT_F9[]  = {27, 91, 50, 48, 59, 50, 126, '\0'};
static constexpr char SHIFT_F10[] = {27, 91, 50, 49, 59, 50, 126, '\0'};
static constexpr char SHIFT_F11[] = {27, 91, 50, 51, 59, 50, 126, '\0'};
static constexpr char SHIFT_F12[] = {27, 91, 50, 52, 59, 50, 126, '\0'};
}  // namespace xterm_seq

const KeyboardHandlerUnixImpl::KeyMap KeyboardHandlerUnixImpl::DEFAULT_STATIC_KEY_MAP[]  = {
  {KeyCode::CURSOR_UP,    xterm_seq::CURSOR_UP},
  {KeyCode::CURSOR_DOWN,  xterm_seq::CURSOR_DOWN},
  {KeyCode::CURSOR_RIGHT, xterm_seq::CURSOR_ONE_STEP_RIGHT},
  {KeyCode::CURSOR_LEFT,  xterm_seq::CURSOR_ONE_STEP_LEFT},
  {KeyCode::SPACE,        xterm_seq::SPACE},
  {KeyCode::ESCAPE,       xterm_seq::ESC},
  {KeyCode::ENTER,        xterm_seq::ENTER},
  {KeyCode::BACK_SPACE,   xterm_seq::BACK_SPACE},

  {KeyCode::DELETE,       xterm_seq::DELETE},
  {KeyCode::END,          xterm_seq::END},
  {KeyCode::PG_DOWN,      xterm_seq::PG_DOWN},
  {KeyCode::PG_UP,        xterm_seq::PG_UP},
  {KeyCode::HOME,         xterm_seq::HOME},
  {KeyCode::INSERT,       xterm_seq::INSERT},

  {KeyCode::F1,           xterm_seq::F1},
  {KeyCode::F2,           xterm_seq::F2},
  {KeyCode::F3,           xterm_seq::F3},
  {KeyCode::F4,           xterm_seq::F4},
  {KeyCode::F5,           xterm_seq::F5},
  {KeyCode::F6,           xterm_seq::F6},
  {KeyCode::F7,           xterm_seq::F7},
  {KeyCode::F8,           xterm_seq::F8},
  {KeyCode::F9,           xterm_seq::F9},
  {KeyCode::F10,          xterm_seq::F10},
  {KeyCode::F11,          xterm_seq::F11},
  {KeyCode::F12,          xterm_seq::F12},

  {KeyCode::SHIFT_F1,     xterm_seq::SHIFT_F1},
  {KeyCode::SHIFT_F2,     xterm_seq::SHIFT_F2},
  {KeyCode::SHIFT_F3,     xterm_seq::SHIFT_F3},
  {KeyCode::SHIFT_F4,     xterm_seq::SHIFT_F4},
  {KeyCode::SHIFT_F5,     xterm_seq::SHIFT_F5},
  {KeyCode::SHIFT_F6,     xterm_seq::SHIFT_F6},
  {KeyCode::SHIFT_F7,     xterm_seq::SHIFT_F7},
  {KeyCode::SHIFT_F8,     xterm_seq::SHIFT_F8},
  {KeyCode::SHIFT_F9,     xterm_seq::SHIFT_F9},
  {KeyCode::SHIFT_F10,    xterm_seq::SHIFT_F10},
  {KeyCode::SHIFT_F11,    xterm_seq::SHIFT_F11},
  {KeyCode::SHIFT_F12,    xterm_seq::SHIFT_F12},

  {KeyCode::A,            "a"},
  {KeyCode::B,            "b"},
  {KeyCode::C,            "c"},
  {KeyCode::D,            "d"},
  {KeyCode::E,            "e"},
  {KeyCode::F,            "f"},
  {KeyCode::G,            "g"},
  {KeyCode::H,            "h"},
  {KeyCode::I,            "i"},
  {KeyCode::J,            "j"},
  {KeyCode::K,            "k"},
  {KeyCode::L,            "l"},
  {KeyCode::M,            "m"},
  {KeyCode::N,            "n"},
  {KeyCode::O,            "o"},
  {KeyCode::P,            "p"},
  {KeyCode::Q,            "q"},
  {KeyCode::R,            "r"},
  {KeyCode::S,            "s"},
  {KeyCode::T,            "t"},
  {KeyCode::U,            "u"},
  {KeyCode::V,            "v"},
  {KeyCode::W,            "w"},
  {KeyCode::X,            "x"},
  {KeyCode::Y,            "y"},
  {KeyCode::Z,            "z"},

  {KeyCode::CAPITAL_A,    "A"},
  {KeyCode::CAPITAL_B,    "B"},
  {KeyCode::CAPITAL_C,    "C"},
  {KeyCode::CAPITAL_D,    "D"},
  {KeyCode::CAPITAL_E,    "E"},
  {KeyCode::CAPITAL_F,    "F"},
  {KeyCode::CAPITAL_G,    "G"},
  {KeyCode::CAPITAL_H,    "H"},
  {KeyCode::CAPITAL_I,    "I"},
  {KeyCode::CAPITAL_J,    "J"},
  {KeyCode::CAPITAL_K,    "K"},
  {KeyCode::CAPITAL_L,    "L"},
  {KeyCode::CAPITAL_M,    "M"},
  {KeyCode::CAPITAL_N,    "N"},
  {KeyCode::CAPITAL_O,    "O"},
  {KeyCode::CAPITAL_P,    "P"},
  {KeyCode::CAPITAL_Q,    "Q"},
  {KeyCode::CAPITAL_R,    "R"},
  {KeyCode::CAPITAL_S,    "S"},
  {KeyCode::CAPITAL_T,    "T"},
  {KeyCode::CAPITAL_U,    "U"},
  {KeyCode::CAPITAL_V,    "V"},
  {KeyCode::CAPITAL_W,    "W"},
  {KeyCode::CAPITAL_X,    "X"},
  {KeyCode::CAPITAL_Y,    "Y"},
  {KeyCode::CAPITAL_Z,    "Z"},

  {KeyCode::NUMBER_1,     "1"},
  {KeyCode::NUMBER_2,     "2"},
  {KeyCode::NUMBER_3,     "3"},
  {KeyCode::NUMBER_4,     "4"},
  {KeyCode::NUMBER_5,     "5"},
  {KeyCode::NUMBER_6,     "6"},
  {KeyCode::NUMBER_7,     "7"},
  {KeyCode::NUMBER_8,     "8"},
  {KeyCode::NUMBER_9,     "9"},
  {KeyCode::NUMBER_0,     "0"},
  {KeyCode::MINUS,        "-"},
  {KeyCode::EQUAL_SIGN,   "="},
};
/* *INDENT-ON* */

const size_t KeyboardHandlerUnixImpl::STATIC_KEY_MAP_LENGTH =
  sizeof(KeyboardHandlerUnixImpl::DEFAULT_STATIC_KEY_MAP) / sizeof(KeyboardHandlerUnixImpl::KeyMap);

#endif  // #ifndef _WIN32
