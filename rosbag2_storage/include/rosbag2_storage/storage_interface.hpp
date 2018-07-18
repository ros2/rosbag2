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

#ifndef ROSBAG2_STORAGE__STORAGE_INTERFACE_HPP_
#define ROSBAG2_STORAGE__STORAGE_INTERFACE_HPP_

#include <string>

#include "rosbag2_storage/storage_handle.hpp"
#include "rosbag2_storage/visibility_control.h"

namespace rosbag2_storage
{

/// Abstract base class for the storage API
/**
 * Every storage plugin shall implement this interface
 * in order to be able to get correctly loaded by the storage API.
 */
class StorageInterface
{
public:
  ROSBAG2_STORAGE_PUBLIC
  StorageInterface() = default;

  ROSBAG2_STORAGE_PUBLIC
  virtual ~StorageInterface() = default;

  /// Open the bag file given the specified path
  /**
   * The function opens the bag file and returns a handle to it.
   * \param file_path the fqdn path to the bag to be loaded
   */
  ROSBAG2_STORAGE_PUBLIC
  virtual
  StorageHandle
  open(const std::string & file_path) = 0;

  /// Close the bag
  /**
   * The function closes the bag file which was previously opened by \sa open.
   * \param storage_handle is the returned handle after opening the bag.
   */
  ROSBAG2_STORAGE_PUBLIC
  virtual
  bool
  close(StorageHandle & storage_handle) = 0;

  ROSBAG2_STORAGE_PUBLIC
  virtual
  void
  set_storage_identifier(const std::string & storage_identifier) = 0;

  /// Get the identifier for this storage handle
  /**
   * The function returns the unique identifier for this storage format
   * \return string identifying the storage format
   */
  ROSBAG2_STORAGE_PUBLIC
  virtual
  std::string
  get_storage_identifier() = 0;
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__STORAGE_INTERFACE_HPP_
