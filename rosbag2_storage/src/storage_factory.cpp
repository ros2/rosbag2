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

#include "rosbag2_storage/storage_factory.hpp"

#include <memory>
#include <string>

#include "./impl/storage_factory_impl.hpp"

namespace rosbag2_storage
{

StorageFactory::StorageFactory()
: impl_(new StorageFactoryImpl())
{
}

StorageFactory::~StorageFactory()
{
  delete impl_;
}

ReadWriteStorageSharedPtr StorageFactory::get_read_write_storage(
  const std::string & storage_id, const std::string & uri)
{
  return impl_->get_read_write_storage(storage_id, uri);
}

ReadOnlyStorageSharedPtr StorageFactory::get_read_only_storage(
  const std::string & storage_id, const std::string & uri)
{
  return impl_->get_read_only_storage(storage_id, uri);
}

}  // namespace rosbag2_storage
