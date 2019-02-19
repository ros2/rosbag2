// Copyright 2019, Denso ADAS Engineering Services GmbH.
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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_SEQUENTIAL_STORAGE_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_SEQUENTIAL_STORAGE_HPP_

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "sqlite_storage.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_storage_plugins
{

class ROSBAG2_STORAGE_DEFAULT_PLUGINS_PUBLIC SqliteSequentialStorage
  : public SqliteStorage
{
public:
  SqliteSequentialStorage() = default;
  ~SqliteSequentialStorage() override = default;

  void open(
    const std::string & uri,
    rosbag2_storage::storage_interfaces::IOFlag io_flag =
    rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;

  rosbag2_storage::BagMetadata get_metadata() override;

  const std::string get_storage_identifier() const override;

protected:
  std::shared_ptr<rcutils_uint8_array_t> read_data() override;
  std::shared_ptr<rcutils_uint8_array_t> get_serialized_data(
    std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message) override;

private:
#pragma pack(push, 1)
  struct MetaInformation
  {
    uint64_t offset;
    uint64_t size;
    uint32_t file_index;
  };
#pragma pack(pop)  // back to whatever the previous packing mode was

  struct BinarySequentialFile
  {
    std::string path;
    std::fstream file_stream;
    std::mutex lock;

    explicit BinarySequentialFile(const std::string & _path)
    : path(_path)
    {
    }
  };

  std::vector<std::unique_ptr<BinarySequentialFile>> data_files_;
};

}  // namespace rosbag2_storage_plugins

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_SEQUENTIAL_STORAGE_HPP_
