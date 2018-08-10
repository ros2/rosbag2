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


#ifndef ROSBAG2_STORAGE__WRITABLE_STORAGE_HPP_
#define ROSBAG2_STORAGE__WRITABLE_STORAGE_HPP_

#include <string>

#include "rosbag2_storage/visibility_control.hpp"

namespace rosbag2_storage
{

class ROSBAG2_STORAGE_PUBLIC WritableStorage
{
public:
  /**
   * The destructor should be implemented and used to close the storage file
   */
  virtual ~WritableStorage() = 0;

  /** Open the specified resource
   * @param uri The identifier of the storage to be opened
   */
  virtual void open(const std::string & uri) = 0;

  /**
   * Open a new topic to be written to
   */
  virtual bool create_topic() = 0;

  /**
   * Write the data to storage
   * @param message String to write
   */
  virtual bool write(std::string message) = 0;
};

inline WritableStorage::~WritableStorage() = default;

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__WRITABLE_STORAGE_HPP_
