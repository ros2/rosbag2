// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_COMPRESSION_ZSTD__VISIBILITY_CONTROL_HPP_
#define ROSBAG2_COMPRESSION_ZSTD__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define ROSBAG2_COMPRESSION_ZSTD_EXPORT __attribute__ ((dllexport))
    #define ROSBAG2_COMPRESSION_ZSTD_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSBAG2_COMPRESSION_ZSTD_EXPORT __declspec(dllexport)
    #define ROSBAG2_COMPRESSION_ZSTD_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSBAG2_COMPRESSION_ZSTD_BUILDING_DLL
    #define ROSBAG2_COMPRESSION_ZSTD_PUBLIC ROSBAG2_COMPRESSION_ZSTD_EXPORT
  #else
    #define ROSBAG2_COMPRESSION_ZSTD_PUBLIC ROSBAG2_COMPRESSION_ZSTD_IMPORT
  #endif
  #define ROSBAG2_COMPRESSION_ZSTD_PUBLIC_TYPE ROSBAG2_COMPRESSION_ZSTD_PUBLIC
  #define ROSBAG2_COMPRESSION_ZSTD_LOCAL
#else
#define ROSBAG2_COMPRESSION_ZSTD_EXPORT __attribute__ ((visibility("default")))
#define ROSBAG2_COMPRESSION_ZSTD_IMPORT
#if __GNUC__ >= 4
#define ROSBAG2_COMPRESSION_ZSTD_PUBLIC __attribute__ ((visibility("default")))
#define ROSBAG2_COMPRESSION_ZSTD_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define ROSBAG2_COMPRESSION_ZSTD_PUBLIC
    #define ROSBAG2_COMPRESSION_ZSTD_LOCAL
#endif
#define ROSBAG2_COMPRESSION_ZSTD_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROSBAG2_COMPRESSION_ZSTD__VISIBILITY_CONTROL_HPP_
