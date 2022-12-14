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

#ifndef ROSBAG2_CPP__PLUGINS__PLUGIN_UTILS_HPP_
#define ROSBAG2_CPP__PLUGINS__PLUGIN_UTILS_HPP_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "pluginlib/class_loader.hpp"

namespace rosbag2_cpp
{
namespace plugins
{

template<typename InterfaceT>
class PluginInfo
{
public:
  PluginInfo()
  : loader_(InterfaceT::get_package_name(), InterfaceT::get_base_class_name())
  {}

  std::unordered_set<std::string> get_declared_classes()
  {
    const auto class_list = loader_.getDeclaredClasses();
    return std::unordered_set<std::string>(class_list.begin(), class_list.end());
  }

  std::string package_for_class(const std::string & class_name)
  {
    return loader_.getClassPackage(class_name);
  }

protected:
  pluginlib::ClassLoader<InterfaceT> loader_;
};

template<typename InterfaceT>
std::unordered_set<std::string> get_class_plugins()
{
  PluginInfo<InterfaceT> plugin_info;
  return plugin_info.get_declared_classes();
}
}  // namespace plugins
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__PLUGINS__PLUGIN_UTILS_HPP_
