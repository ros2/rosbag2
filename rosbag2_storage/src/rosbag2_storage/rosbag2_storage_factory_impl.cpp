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

#include "impl/storage_interface_loader.hpp"

namespace rosbag2_storage
{

using rosbag2_storage::storage_interfaces::ReadOnlyInterface;
using rosbag2_storage::storage_interfaces::ReadWriteInterface;

Rosbag2StorageFactoryImpl::Rosbag2StorageFactoryImpl()
: impl_(std::make_unique<StorageInterfaceLoader>())
{}

// needed explicit destructor because of unique_ptr for pimpl
Rosbag2StorageFactoryImpl::~Rosbag2StorageFactoryImpl() = default;

template<typename InterfaceT, storage_interfaces::IOFlag flag>
std::shared_ptr<InterfaceT>
Rosbag2StorageFactoryImpl::open_instance(const std::string & storage_id, const std::string & uri)
{
  auto instance = impl_->load_instance<InterfaceT>(storage_id);

  if (instance == nullptr) {
    return nullptr;
  }

  try {
    instance->open(uri, flag);
    return instance;
  } catch (const std::runtime_error & ex) {
    ROSBAG2_STORAGE_LOG_ERROR_STREAM(
      "Could not open '" << uri << "' with '" << storage_id << "'. Error: " << ex.what());
    return nullptr;
  }
}

std::shared_ptr<ReadOnlyInterface> Rosbag2StorageFactoryImpl::open_read_only(
  const std::string & uri, const std::string & storage_id)
{
  // try to load the instance as read_only interface
  auto instance = open_instance<ReadOnlyInterface>(storage_id, uri);
  // try to load as read_write if not successful
  if (instance == nullptr) {
    instance = open_instance<ReadWriteInterface, storage_interfaces::IOFlag::READ_ONLY>(
      storage_id, uri);
  }

  if (instance == nullptr) {
    ROSBAG2_STORAGE_LOG_ERROR_STREAM(
      "Could not load/open plugin with storage id '" << storage_id << "'.");
  }

  return instance;
}

std::shared_ptr<ReadWriteInterface> Rosbag2StorageFactoryImpl::open_read_write(
  const std::string & uri, const std::string & storage_id)
{
  auto instance = open_instance<ReadWriteInterface>(storage_id, uri);

  if (instance == nullptr) {
    ROSBAG2_STORAGE_LOG_ERROR_STREAM(
      "Could not load/open plugin with storage id '" << storage_id << "'.");
  }

  return instance;
}

std::shared_ptr<MetadataIo> Rosbag2StorageFactoryImpl::metadata_io()
{
  if (!metadata_io_) {
    metadata_io_ = std::make_shared<MetadataIoImpl>();
  }
  return metadata_io_;
}

}  // namespace rosbag2_storage
