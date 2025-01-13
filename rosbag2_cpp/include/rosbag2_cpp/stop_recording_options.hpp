// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_CPP__STOP_RECORDING_OPTIONS_HPP_
#define ROSBAG2_CPP__STOP_RECORDING_OPTIONS_HPP_

#include <cstdint>

namespace rosbag2_cpp
{

struct StopRecordingOptions
{
  uint64_t max_duration;
  uint64_t max_size;
  uint64_t max_messages;
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__STOP_RECORDING_OPTIONS_HPP_
