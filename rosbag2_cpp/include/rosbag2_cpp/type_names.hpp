// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_CPP__TYPE_NAMES_HPP_
#define ROSBAG2_CPP__TYPE_NAMES_HPP_

#include <string>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "test_msgs/msg/basic_types.hpp"


namespace rosbag2_cpp
{

/// \brief Simple function to return names in "package/type" format
template<class T>
std::string type_names();

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__TYPE_NAMES_HPP_
