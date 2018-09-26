// Copyright 2018,  Open Source Robotics Foundation, Inc.
// Copyright 2018,  Bosch Software Innovations GmbH.
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

#include "rosbag2_storage/rosbag2_storage_factory_impl.hpp"

#include <memory>
#include <string>

#include "rosbag2_storage/metadata_io_impl.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

#include "./impl/storage_factory_impl.hpp"

namespace rosbag2_storage
{

using rosbag2_storage::storage_interfaces::ReadOnlyInterface;
using rosbag2_storage::storage_interfaces::ReadWriteInterface;

Rosbag2StorageFactoryImpl::Rosbag2StorageFactoryImpl()
: impl_(std::make_unique<StorageFactoryImpl>())
{}

// needed explicit destructor because of unique_ptr for pimpl
Rosbag2StorageFactoryImpl::~Rosbag2StorageFactoryImpl() = default;

std::shared_ptr<ReadOnlyInterface> Rosbag2StorageFactoryImpl::open_read_only(
  const std::string & uri, const std::string & storage_id)
{
  return impl_->open_read_only(uri, storage_id);
}

std::shared_ptr<ReadWriteInterface> Rosbag2StorageFactoryImpl::open_read_write(
  const std::string & uri, const std::string & storage_id)
{
  return impl_->open_read_write(uri, storage_id);
}

std::shared_ptr<MetadataIo> Rosbag2StorageFactoryImpl::metadata_io()
{
  if (!metadata_io_) {
    metadata_io_ = std::make_shared<MetadataIoImpl>();
  }
  return metadata_io_;
}

}  // namespace rosbag2_storage
