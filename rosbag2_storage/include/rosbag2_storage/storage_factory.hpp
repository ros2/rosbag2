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

#include "rosbag2_storage/storage_interface.hpp"
#include "rosbag2_storage/visibility_control.h"

namespace rosbag2_storage
{

class StorageFactoryImpl;

/// Factory to create instances for various storage interfaces
class StorageFactory
{
public:
  explicit StorageFactory(const std::string & storage_identifier);

  virtual ~StorageFactory();

  /// Static function to create \sa StorageInterface given the storage_identifier
  /**
   * The function returns the StorageInterface base class for a specific identifier.
   * \param storage_identifier unique string indicating which storage format to use.
   */
  ROSBAG2_STORAGE_PUBLIC
  std::shared_ptr<StorageInterface>
  get_storage_interface();

private:
  std::unique_ptr<StorageFactoryImpl> impl_;
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__STORAGE_FACTORY_HPP_
