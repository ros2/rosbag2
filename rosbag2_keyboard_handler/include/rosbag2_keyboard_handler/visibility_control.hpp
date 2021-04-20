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

#ifndef ROSBAG2_KEYBOARD_HANDLER__VISIBILITY_CONTROL_HPP_
#define ROSBAG2_KEYBOARD_HANDLER__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KEYBOARD_HANDLER_EXPORT __attribute__ ((dllexport))
    #define KEYBOARD_HANDLER_IMPORT __attribute__ ((dllimport))
  #else
    #define KEYBOARD_HANDLER_EXPORT __declspec(dllexport)
    #define KEYBOARD_HANDLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef KEYBOARD_HANDLER_BUILDING_LIBRARY
    #define KEYBOARD_HANDLER_PUBLIC KEYBOARD_HANDLER_EXPORT
  #else
    #define KEYBOARD_HANDLER_PUBLIC KEYBOARD_HANDLER_IMPORT
  #endif
  #define KEYBOARD_HANDLER_PUBLIC_TYPE KEYBOARD_HANDLER_PUBLIC
  #define KEYBOARD_HANDLER_LOCAL
#else
  #define KEYBOARD_HANDLER_EXPORT __attribute__ ((visibility("default")))
  #define KEYBOARD_HANDLER_IMPORT
  #if __GNUC__ >= 4
    #define KEYBOARD_HANDLER_PUBLIC __attribute__ ((visibility("default")))
    #define KEYBOARD_HANDLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KEYBOARD_HANDLER_PUBLIC
    #define KEYBOARD_HANDLER_LOCAL
  #endif
  #define KEYBOARD_HANDLER_PUBLIC_TYPE
#endif

#endif  // ROSBAG2_KEYBOARD_HANDLER__VISIBILITY_CONTROL_HPP_
