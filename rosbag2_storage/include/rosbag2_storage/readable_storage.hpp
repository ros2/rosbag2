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

#include "visibility_control.hpp"

namespace rosbag2_storage
{

/**
 * Struct to hold the storage information.
 */
struct BagInfo
{
  std::string uri;
  // TODO(greimela-si/botteroa-si): Add remaining info fields.
};

class ROSBAG2_STORAGE_PUBLIC ReadableStorage
{
public:
  /**
   * The destructor should be implemented and used to close the storage file.
   */
  virtual ~ReadableStorage() = 0;

  /** Open the specified resource for reading.
   * @param uri The identifier of the storage to be opened.
   */
  virtual void open_for_reading(const std::string & uri) = 0;

  // TODO(greimela-si/botteroa-si): find out correct parameter type.
  /**
   * Read the next data set from storage.
   * @param buffer .
   * @param size Size of the data in bytes.
   */
  virtual void read_next(char * buffer, size_t size) = 0;

  /**
   * Retrieve the storage information.
   * @return The BagInfo of the storage.
   */
  virtual BagInfo info() = 0;
};

inline ReadableStorage::~ReadableStorage() = default;

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__READABLE_STORAGE_HPP_
