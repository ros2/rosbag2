// Copyright 2023 Patrick Roncagliolo.
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

#ifndef ROSBAG2_TRANSPORT__UTILS__PARAM_UTILS_HPP_
#define ROSBAG2_TRANSPORT__UTILS__PARAM_UTILS_HPP_

#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace rosbag2_transport
{

namespace param_utils
{
rcl_interfaces::msg::ParameterDescriptor int_param_description(
  std::string description, int64_t min,
  int64_t max);

rcl_interfaces::msg::ParameterDescriptor float_param_description(
  std::string description, float min,
  float max);

}  // namespace param_utils
}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__UTILS__PARAM_UTILS_HPP_
