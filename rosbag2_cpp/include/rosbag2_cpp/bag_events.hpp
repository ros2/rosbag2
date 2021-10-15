// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_CPP__BAG_EVENTS_HPP_
#define ROSBAG2_CPP__BAG_EVENTS_HPP_

#include <functional>

namespace rosbag2_cpp
{

enum class BagEvent {
  SplitOutputFile,
};

struct BagEventCallback {
  std::function<void ()> function;
  BagEvent type;
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__BAG_EVENTS_HPP_
