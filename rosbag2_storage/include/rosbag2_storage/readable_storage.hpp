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

#ifndef ROSBAG2_STORAGE__READABLE_STORAGE_HPP_
#define ROSBAG2_STORAGE__READABLE_STORAGE_HPP_

#include <string>

#include "bag_info.hpp"
#include "visibility_control.hpp"

namespace rosbag2_storage
{

class ROSBAG2_STORAGE_PUBLIC ReadableStorage
{
public:
  /**
   * The destructor should be implemented and used to close the storage file.
   */
  virtual ~ReadableStorage() = 0;

  /** Open the specified resource.
   * @param uri The identifier of the storage to be opened.
   */
  virtual void open(const std::string & uri) = 0;

  /**
   * Read the next data set from storage.
   * @param message String to save the message.
   */
  virtual bool read_next(std::string & message) = 0;

  /**
   * Retrieve the storage information.
   * @return The BagInfo of the storage.
   */
  virtual BagInfo info() = 0;
};

inline ReadableStorage::~ReadableStorage() = default;

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__READABLE_STORAGE_HPP_
