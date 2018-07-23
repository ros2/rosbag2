/*
 *  Copyright (c) 2018,  Bosch Software Innovations GmbH.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef ROSBAG2__STORAGE__STORAGE_FACTORY_HPP_
#define ROSBAG2__STORAGE__STORAGE_FACTORY_HPP_

#include <memory>
#include <string>

#include "readable_storage.hpp"
#include "writable_storage.hpp"

namespace rosbag2
{
class StorageFactory
{
public:
  std::unique_ptr<ReadableStorage> get_for_reading(const std::string & file_name);
  std::unique_ptr<WritableStorage> get_for_writing(const std::string & file_name);
};

}  // namespace rosbag2

#endif  // ROSBAG2__STORAGE__STORAGE_FACTORY_HPP_
