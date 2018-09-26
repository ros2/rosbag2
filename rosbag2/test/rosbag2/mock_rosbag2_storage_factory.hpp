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

#include <memory>
#include <string>

#include "rosbag2_storage/rosbag2_storage_factory.hpp"
#include "mock_storage.hpp"

#ifndef ROSBAG2__MOCK_ROSBAG2_STORAGE_FACTORY_HPP_
#define ROSBAG2__MOCK_ROSBAG2_STORAGE_FACTORY_HPP_

class MockRosbag2StorageFactory : public rosbag2_storage::Rosbag2StorageFactory
{
public:
  explicit MockRosbag2StorageFactory(std::shared_ptr<rosbag2_storage::MetadataIo> metadata_io)
  : metadata_io_(metadata_io) {}

  ~MockRosbag2StorageFactory() override = default;

  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface>
  open_read_only(const std::string & uri, const std::string & storage_id) override
  {
    (void) uri;
    (void) storage_id;
    return std::make_shared<MockStorage>();
  }

  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface>
  open_read_write(const std::string & uri, const std::string & storage_id) override
  {
    (void) uri;
    (void) storage_id;
    return std::make_shared<MockStorage>();
  }

  std::shared_ptr<rosbag2_storage::MetadataIo> metadata_io() override
  {
    return metadata_io_;
  }

private:
  std::shared_ptr<rosbag2_storage::MetadataIo> metadata_io_;
};

#endif  // ROSBAG2__MOCK_ROSBAG2_STORAGE_FACTORY_HPP_
