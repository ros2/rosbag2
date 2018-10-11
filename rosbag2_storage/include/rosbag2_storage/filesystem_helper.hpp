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

#ifdef _WIN32
# include <windows.h>
#else
# include <dirent.h>
#endif

#include <sys/stat.h>

#include <cstring>
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
      auto start = path.rfind(separator, last_separator - 1);
      return path.substr(start + 1, last_separator - (start + 1));
    } else {
      return path.substr(
        last_separator == std::string::npos ? 0 : last_separator + 1, path.length());
    }
  }

  /**
   * Calculates the size of a directory by summarizing the file size of all files
   * Note: This operation is not recursive
   * \param directory_path The directory path to calculate the size of
   * \return The size of the directory
   */
  static size_t calculate_directory_size(const std::string & directory_path)
  {
    size_t dir_size = 0;
#ifdef _WIN32
    WIN32_FIND_DATA data;
    HANDLE handle = FindFirstFile(concat({directory_path, "*"}).c_str(), &data);
    if (handle != INVALID_HANDLE_VALUE) {
      do {
        if (strcmp(data.cFileName, ".") != 0 && strcmp(data.cFileName, "..") != 0) {
          dir_size += get_file_size(concat({directory_path, data.cFileName}));
        }
      } while (FindNextFile(handle, &data));
      FindClose(handle);
    }
    return dir_size;
#else
    DIR * dir;
    dirent * entry;
    if ((dir = opendir(directory_path.c_str())) != nullptr) {
      while ((entry = readdir(dir)) != nullptr) {
        if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
          dir_size += get_file_size(concat({directory_path, entry->d_name}));
        }
      }
      closedir(dir);
    }
    return dir_size;
#endif
  }

  static size_t get_file_size(const std::string & file_path)
  {
    struct stat stat_buffer {};
    int rc = stat(file_path.c_str(), &stat_buffer);
    return rc == 0 ? static_cast<size_t>(stat_buffer.st_size) : 0;
  }
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__FILESYSTEM_HELPER_HPP_
