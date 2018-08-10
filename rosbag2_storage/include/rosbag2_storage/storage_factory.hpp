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

#ifndef ROSBAG2_STORAGE__STORAGE_FACTORY_HPP_
#define ROSBAG2_STORAGE__STORAGE_FACTORY_HPP_

#include <memory>
#include <string>

#include "rosbag2_storage/storage.hpp"
#include "rosbag2_storage/writable_storage.hpp"
#include "rosbag2_storage/readable_storage.hpp"

#include "rosbag2_storage/visibility_control.hpp"

namespace rosbag2_storage
{

class StorageFactoryImpl;

using ReadOnlyStorageSharedPtr = std::shared_ptr<ReadableStorage>;
using WriteOnlyStorageSharedPtr = std::shared_ptr<WritableStorage>;
using StorageSharedPtr = std::shared_ptr<Storage>;

/// Factory to create instances of various storage interfaces
class ROSBAG2_STORAGE_PUBLIC StorageFactory
{
public:
  StorageFactory();
  virtual ~StorageFactory();

  template<class T>
  T get_storage(const std::string & storage_id, const std::string & uri);

private:
  std::unique_ptr<StorageFactoryImpl> impl_;
};

template<>
ROSBAG2_STORAGE_PUBLIC
WriteOnlyStorageSharedPtr StorageFactory::get_storage(
  const std::string & storage_id, const std::string & uri);

template<>
ROSBAG2_STORAGE_PUBLIC
ReadOnlyStorageSharedPtr StorageFactory::get_storage(
  const std::string & storage_id, const std::string & uri);

template<>
ROSBAG2_STORAGE_PUBLIC
StorageSharedPtr StorageFactory::get_storage(
  const std::string & storage_id, const std::string & uri);

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__STORAGE_FACTORY_HPP_
