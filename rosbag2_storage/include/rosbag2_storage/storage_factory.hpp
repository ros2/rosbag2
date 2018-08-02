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

#include "rosbag2_storage/writable_storage.hpp"
#include "rosbag2_storage/readable_storage.hpp"

#include "rosbag2_storage/visibility_control.h"

namespace rosbag2_storage
{

class StorageFactoryImpl;

/// Factory to create instances of various storage interfaces
class StorageFactory
{
public:
  StorageFactory();
  virtual ~StorageFactory() = default;

  virtual std::unique_ptr<WritableStorage> get_for_writing(
    const std::string & storage_id, const std::string & uri);
  virtual std::unique_ptr<ReadableStorage> get_for_reading(
    const std::string & storage_id, const std::string & uri);

private:
  StorageFactoryImpl * impl_;
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__STORAGE_FACTORY_HPP_
