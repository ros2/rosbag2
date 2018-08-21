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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STORAGE_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STORAGE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "sqlite_wrapper.hpp"

#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

#include "rcutils/types.h"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_storage_plugins
{

class SqliteStorage : public rosbag2_storage::storage_interfaces::ReadWriteInterface
{
public:
  SqliteStorage();
  explicit SqliteStorage(std::shared_ptr<SqliteWrapper> database);
  ~SqliteStorage() override = default;

  void open(
    const std::string & uri,
    rosbag2_storage::storage_interfaces::IOFlag io_flag =
    rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;

  void create_topic() override;

  void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message) override;

  bool has_next() override;

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;

  rosbag2_storage::BagInfo info() override;

private:
  void initialize();
  void prepare_for_writing();
  void prepare_for_reading();

  std::shared_ptr<SqliteWrapper> database_;
  rosbag2_storage::BagInfo bag_info_;
  std::shared_ptr<SqliteStatementWrapper> write_statement_;
  std::shared_ptr<SqliteStatementWrapper> read_statement_;
  bool ready_to_read_next_;
};

}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STORAGE_HPP_
