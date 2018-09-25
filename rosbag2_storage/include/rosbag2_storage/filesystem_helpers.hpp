// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_STORAGE__FILESYSTEM_HELPERS_HPP_
#define ROSBAG2_STORAGE__FILESYSTEM_HELPERS_HPP_

#include <string>

namespace rosbag2_storage
{

std::string separator()
{
#ifdef _WIN32
  return "\\";
#else
  return "/";
#endif
}

bool ends_with_separator(const std::string & path)
{
  auto last_separator = path.rfind(separator());
  return last_separator == path.size() - 1;
}

std::string get_leaf_directory(const std::string & path)
{
  auto last_separator = path.rfind(separator());
  if (last_separator == path.size() - 1) {
    auto start = path.rfind(separator(), path.size() - 1);
    return path.substr(start + 1, path.size() - 1);
  } else {
    return path.substr(last_separator == std::string::npos ? 0 : last_separator, path.length());
  }
}

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__FILESYSTEM_HELPERS_HPP_
