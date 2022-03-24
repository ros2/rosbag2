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
#ifndef ROSBAG2_STORAGE__YAML_HPP_
#define ROSBAG2_STORAGE__YAML_HPP_

#include <string>
#include <unordered_map>

#ifdef _WIN32
// This is necessary because of a bug in yaml-cpp's cmake
#define YAML_CPP_DLL
// This is necessary because yaml-cpp does not always use dllimport/dllexport consistently
# pragma warning(push)
# pragma warning(disable:4251)
# pragma warning(disable:4275)
#endif
#include "yaml-cpp/yaml.h"
#ifdef _WIN32
# pragma warning(pop)
#endif

namespace YAML
{

/// If "node" has a field named "field", then assign its value to "assign_to" destination.
template<typename T>
void optional_assign(const Node & node, std::string field, T & assign_to)
{
  if (node[field]) {
    assign_to = node[field].as<T>();
  }
}

template<>
struct convert<std::unordered_map<std::string, std::string>>
{
  static Node encode(const std::unordered_map<std::string, std::string> & custom_data)
  {
    Node node;
    for (const auto & it : custom_data) {
      node[it.first] = it.second;
    }
    return node;
  }

  static bool decode(const Node & node, std::unordered_map<std::string, std::string> & custom_data)
  {
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
      custom_data.emplace(it->first.as<std::string>(), it->second.as<std::string>());
    }
    return true;
  }
};

}  // namespace YAML

#endif  // ROSBAG2_STORAGE__YAML_HPP_
