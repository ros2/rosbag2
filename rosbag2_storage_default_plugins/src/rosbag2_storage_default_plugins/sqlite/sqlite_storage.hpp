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
 *  limitations under the License.≤
 */

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STORAGE_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STORAGE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "sqlite_wrapper.hpp"

#include "rosbag2_storage/writable_storage.hpp"
#include "rosbag2_storage/readable_storage.hpp"

namespace rosbag2_storage_plugins
{

class SqliteStorage
  : public rosbag2_storage::WritableStorage, public rosbag2_storage::ReadableStorage
{
public:
  SqliteStorage();
  explicit SqliteStorage(std::shared_ptr<SqliteWrapper> database);
  ~SqliteStorage() override = default;

  void open_for_writing(const std::string & uri) override;

  void open_for_reading(const std::string & uri) override;

  bool write(void * data, size_t size) override;

  bool read_next(void * buffer, size_t & size) override;

  rosbag2_storage::BagInfo info() override;

private:
  void initialize();

  std::shared_ptr<SqliteWrapper> database_;
  size_t counter_;
};

}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STORAGE_HPP_
