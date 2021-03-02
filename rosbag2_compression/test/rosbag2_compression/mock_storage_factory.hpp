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

#ifndef ROSBAG2_COMPRESSION__MOCK_STORAGE_FACTORY_HPP_
#define ROSBAG2_COMPRESSION__MOCK_STORAGE_FACTORY_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/storage_factory_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

class MockStorageFactory : public rosbag2_storage::StorageFactoryInterface
{
public:
  MOCK_METHOD(
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface>,
    open_read_only,
    (const rosbag2_storage::StorageOptions &),
    (override));
  MOCK_METHOD(
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface>,
    open_read_write,
    (const rosbag2_storage::StorageOptions &),
    (override));
  std::vector<std::string> get_declared_read_only_plugins() const override {return {};}
  std::vector<std::string> get_declared_read_write_plugins() const override {return {};}
};

#endif  // ROSBAG2_COMPRESSION__MOCK_STORAGE_FACTORY_HPP_
