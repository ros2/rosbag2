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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_EXCEPTION_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_EXCEPTION_HPP_

#include <stdexcept>
#include <string>

#include "rosbag2_storage_default_plugins/visibility_control.hpp"

// ignore incorrect warning when deriving from standard library types
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4275)
#endif

namespace rosbag2_storage_plugins
{

class ROSBAG2_STORAGE_DEFAULT_PLUGINS_PUBLIC SqliteException : public std::runtime_error
{
public:
  explicit SqliteException(const std::string & message)
  : runtime_error(message), sqlite_return_code_(-1) {}

  SqliteException(const std::string & message, int sqlite_return_code)
  : runtime_error(message), sqlite_return_code_(sqlite_return_code) {}

  /// Return the sqlite return code associated with the Exception, or -1 if none was given.
  int
  get_sqlite_return_code() const
  {
    return sqlite_return_code_;
  }

private:
  int sqlite_return_code_;
};

}  // namespace rosbag2_storage_plugins

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_EXCEPTION_HPP_
