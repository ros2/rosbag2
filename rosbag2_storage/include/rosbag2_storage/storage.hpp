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

#ifndef ROSBAG2_STORAGE__STORAGE_HPP_
#define ROSBAG2_STORAGE__STORAGE_HPP_

#include <string>

#include "visibility_control.hpp"
#include "readable_storage.hpp"
#include "writable_storage.hpp"

namespace rosbag2_storage
{

class ROSBAG2_STORAGE_PUBLIC Storage : public ReadableStorage, public WritableStorage
{
public:
  ~Storage() override;

  void open_for_reading(const std::string & uri) override = 0;

  bool read_next(void * buffer, size_t & size) override = 0;

  void open_for_writing(const std::string & uri) override = 0;

  bool write(void * data, size_t size) override = 0;

  BagInfo info() override = 0;
};

inline Storage::~Storage() = default;

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__STORAGE_HPP_
