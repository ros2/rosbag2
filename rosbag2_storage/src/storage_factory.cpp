// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "rosbag2_storage/storage_factory.hpp"
#include "./impl/storage_factory_impl.hpp"

namespace rosbag2_storage
{

StorageFactory::StorageFactory(const std::string & storage_identifier)
: impl_(new StorageFactoryImpl(storage_identifier))
{
}

StorageFactory::~StorageFactory()
{
}

std::shared_ptr<StorageInterface>
StorageFactory::get_storage_interface()
{
  return impl_->get_instance();
}

}  // namespace rosbag2_storage
