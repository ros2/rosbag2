// Copyright 2020, Robotec.ai sp. z o.o.
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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_PRAGMAS_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_PRAGMAS_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>


namespace rosbag2_storage_plugins
{

class ROSBAG2_STORAGE_DEFAULT_PLUGINS_PUBLIC SqlitePragmas
{
public:
  using pragmas_map_t = std::unordered_map<std::string, std::string>;

  static pragmas_map_t default_pragmas()
  {
    static pragmas_map_t p = {{"schema_version", "PRAGMA schema_version;"}};
    return p;
  }

  // more robust storage settings will cause performace hit on writing,
  // but increase resistance to bagfile data corruption in case of crashes
  static pragmas_map_t robust_writing_pragmas()
  {
    static pragmas_map_t p = {
      {"journal_mode", "PRAGMA journal_mode=WAL;"},
      {"synchronous", "PRAGMA synchronous=NORMAL;"}
    };
    return p;
  }

  static pragmas_map_t optimized_writing_pragmas()
  {
    static pragmas_map_t p = {
      {"journal_mode", "PRAGMA journal_mode=MEMORY;"},
      {"synchronous", "PRAGMA synchronous=OFF;"}
    };
    return p;
  }
};

}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_PRAGMAS_HPP_
