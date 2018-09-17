// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2__TYPESUPPORT_HELPERS_HPP_
#define ROSBAG2__TYPESUPPORT_HELPERS_HPP_

#include <string>
#include <utility>

#include "rosidl_generator_cpp/message_type_support_decl.hpp"

namespace rosbag2
{

std::string get_typesupport_library_path(
  const std::string & package_name, const std::string & typesupport_identifier);

const std::pair<std::string, std::string> extract_type_and_package(const std::string & full_type);

const rosidl_message_type_support_t * get_typesupport(const std::string & type);

}  // namespace rosbag2

#endif  // ROSBAG2__TYPESUPPORT_HELPERS_HPP_
