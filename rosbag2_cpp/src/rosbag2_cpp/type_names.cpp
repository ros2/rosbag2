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

#include <string>

#include "rosbag2_cpp/type_names.hpp"

namespace rosbag2_cpp
{

template<>
std::string type_names<std_msgs::msg::Bool>()
{
  return "std_msgs/Bool";
}
template<>
std::string type_names<std_msgs::msg::Byte>()
{
  return "std_msgs/Byte";
}
template<>
std::string type_names<std_msgs::msg::Char>()
{
  return "std_msgs/Char";
}
template<>
std::string type_names<std_msgs::msg::Float32>()
{
  return "std_msgs/Float32";
}
template<>
std::string type_names<std_msgs::msg::Float64>()
{
  return "std_msgs/Float64";
}
template<>
std::string type_names<std_msgs::msg::Int8>()
{
  return "std_msgs/Int8";
}
template<>
std::string type_names<std_msgs::msg::UInt8>()
{
  return "std_msgs/UInt8";
}
template<>
std::string type_names<std_msgs::msg::Int16>()
{
  return "std_msgs/Int16";
}
template<>
std::string type_names<std_msgs::msg::UInt16>()
{
  return "std_msgs/UInt16";
}
template<>
std::string type_names<std_msgs::msg::Int32>()
{
  return "std_msgs/Int32";
}
template<>
std::string type_names<std_msgs::msg::UInt32>()
{
  return "std_msgs/UInt32";
}
template<>
std::string type_names<std_msgs::msg::Int64>()
{
  return "std_msgs/Int64";
}
template<>
std::string type_names<std_msgs::msg::UInt64>()
{
  return "std_msgs/UInt64";
}

template<>
std::string type_names<test_msgs::msg::BasicTypes>()
{
  return "test_msgs/BasicTypes";
}

}  // namespace rosbag2_cpp
