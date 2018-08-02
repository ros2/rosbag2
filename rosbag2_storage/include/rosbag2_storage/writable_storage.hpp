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

namespace rosbag2_storage
{

class WritableStorage
{
public:
  /**
   * The destructor should be implemented and used to close the storage file.
   */
  virtual ~WritableStorage() = 0;

  /** Open the specified resource for writing.
   * @param uri The identifier of the storage to be opened.
   */
  virtual void open_for_writing(const std::string & uri) = 0;

  /**
   * Write the data to storage.
   * @param data Binary data to write.
   * @param size Size of the data in bytes.
   */
  virtual void write(char * data, size_t size) = 0;
};

inline WritableStorage::~WritableStorage() = default;

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__WRITABLE_STORAGE_HPP_
