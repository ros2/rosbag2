// Copyright 2022 Foxglove Technologies Inc. All Rights Reserved.
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

/* This header must be included by this library's headers which declare symbols
 * that are defined separately. The contents of this header change the visibility
 * of certain symbols, especially for Windows DLL usage.
 */

#ifndef ROSBAG2_STORAGE_MCAP__VISIBILITY_CONTROL_HPP_
#define ROSBAG2_STORAGE_MCAP__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSBAG2_STORAGE_MCAP_EXPORT __attribute__((dllexport))
    #define ROSBAG2_STORAGE_MCAP_IMPORT __attribute__((dllimport))
  #else
    #define ROSBAG2_STORAGE_MCAP_EXPORT __declspec(dllexport)
    #define ROSBAG2_STORAGE_MCAP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSBAG2_STORAGE_MCAP_BUILDING_DLL
    #define ROSBAG2_STORAGE_MCAP_PUBLIC ROSBAG2_STORAGE_MCAP_EXPORT
  #else
    #define ROSBAG2_STORAGE_MCAP_PUBLIC ROSBAG2_STORAGE_MCAP_IMPORT
  #endif
#else
  #define ROSBAG2_STORAGE_MCAP_EXPORT __attribute__((visibility("default")))
  #define ROSBAG2_STORAGE_MCAP_IMPORT
  #if __GNUC__ >= 4
    #define ROSBAG2_STORAGE_MCAP_PUBLIC __attribute__((visibility("default")))
  #else
    #define ROSBAG2_STORAGE_MCAP_PUBLIC
  #endif
#endif

#endif  // ROSBAG2_STORAGE_MCAP__VISIBILITY_CONTROL_HPP_
