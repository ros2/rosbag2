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

#ifndef ROSBAG2_PY__PY_UTILS_HPP_
#define ROSBAG2_PY__PY_UTILS_HPP_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "pluginlib/class_loader.hpp"

namespace rosbag2_py
{
template<typename InterfaceT>
std::unordered_set<std::string> get_class_plugins()
{
  std::string package_name = InterfaceT::get_package_name();
  std::string base_class = InterfaceT::get_base_class_name();
  std::shared_ptr<pluginlib::ClassLoader<InterfaceT>> class_loader =
    std::make_shared<pluginlib::ClassLoader<InterfaceT>>(package_name, base_class);

  std::vector<std::string> plugin_list = class_loader->getDeclaredClasses();
  return std::unordered_set<std::string>(plugin_list.begin(), plugin_list.end());
}
}  // namespace rosbag2_py

#endif  // ROSBAG2_PY__PY_UTILS_HPP_
