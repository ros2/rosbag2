/*
 *  Copyright (c) 2018,  Bosch Software Innovations GmbH.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef ROSBAG2__STORAGE__SQLITE__SQLITE_STORAGE_HPP_
#define ROSBAG2__STORAGE__SQLITE__SQLITE_STORAGE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "sqlite_wrapper.hpp"

#include "../readable_storage.hpp"
#include "../writable_storage.hpp"

namespace rosbag2
{

class SqliteStorage : public WritableStorage, public ReadableStorage
{
public:
  SqliteStorage(const std::string & database_name, bool should_initialize);

  // constructor for testing
  explicit SqliteStorage(std::shared_ptr<SqliteWrapper> database);

  bool write(const std::string & data) override;

  std::vector<std::string> get_messages() override;

private:
  void initialize();

  std::shared_ptr<SqliteWrapper> database_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__STORAGE__SQLITE__SQLITE_STORAGE_HPP_
