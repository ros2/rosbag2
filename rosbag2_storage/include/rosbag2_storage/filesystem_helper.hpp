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

#ifndef ROSBAG2_STORAGE__FILESYSTEM_HELPER_HPP_
#define ROSBAG2_STORAGE__FILESYSTEM_HELPER_HPP_

#include <sstream>
#include <string>

namespace rosbag2_storage
{

class FilesystemHelper
{
public:
#ifdef _WIN32
  static constexpr const char * const separator = "\\";
#else
  static constexpr const char * const separator = "/";
#endif

  /**
   * Concatenates a list of path parts and separates them with a directory separator
   * \param parts
   * \return The concatenated string
   */
  static std::string concat(std::initializer_list<std::string> parts)
  {
    std::stringstream ss;

    auto it = parts.begin();
    for (size_t i = 0; i < parts.size(); ++i) {
      if (i != 0) {
        ss << separator;
      }
      ss << it[i];
    }

    return ss.str();
  }

  /**
   * Returns the name of the folder identified by a file system path
   * \param path The path to a folder
   * \return The name of the folder
   */
  static std::string get_folder_name(const std::string & path)
  {
    auto last_separator = path.rfind(separator);
    if (last_separator == path.size() - 1) {
      auto start = path.rfind(separator, path.size() - 1);
      return path.substr(start + 1, path.size() - 1);
    } else {
      return path.substr(last_separator == std::string::npos ? 0 : last_separator, path.length());
    }
  }

private:
  static bool ends_with_separator(const std::string & path)
  {
    auto last_separator = path.rfind(separator);
    return last_separator == path.size() - 1;
  }
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__FILESYSTEM_HELPER_HPP_
