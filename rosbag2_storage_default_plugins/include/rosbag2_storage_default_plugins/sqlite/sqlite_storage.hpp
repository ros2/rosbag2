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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rcutils/types.h"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.hpp"
#include "rosbag2_storage_default_plugins/visibility_control.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_storage_plugins
{

class ROSBAG2_STORAGE_DEFAULT_PLUGINS_PUBLIC SqliteStorage
  : public rosbag2_storage::storage_interfaces::ReadWriteInterface
{
public:
  SqliteStorage();
  ~SqliteStorage() override = default;

  void open(
    const std::string & uri,
    rosbag2_storage::storage_interfaces::IOFlag io_flag =
    rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;

  void create_topic(const std::string & name, const std::string & type) override;

  void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message) override;

  bool has_next() override;

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;

  std::map<std::string, std::string> get_all_topics_and_types() override;

  rosbag2_storage::BagInfo info() override;

private:
  void initialize();
  void prepare_for_writing();
  void prepare_for_reading();
  void fill_topics_and_types();
  bool database_exists(const std::string & uri);

  using ReadQueryResult = SqliteStatementWrapper::QueryResult<
    std::shared_ptr<rcutils_char_array_t>, rcutils_time_point_value_t, std::string>;

  std::shared_ptr<SqliteWrapper> database_;
  rosbag2_storage::BagInfo bag_info_;
  SqliteStatement write_statement_;
  SqliteStatement read_statement_;
  ReadQueryResult message_result_;
  ReadQueryResult::Iterator current_message_row_;
  std::map<std::string, int> topics_;
  std::map<std::string, std::string> all_topics_and_types_;
};

}  // namespace rosbag2_storage_plugins

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STORAGE_HPP_
