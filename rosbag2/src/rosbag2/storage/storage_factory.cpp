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

#include "storage_factory.hpp"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "sqlite/sqlite_storage.hpp"

namespace rosbag2
{

std::unique_ptr<WritableStorage> StorageFactory::get_for_writing(const std::string & file_name)
{
  std::ifstream infile(file_name);
  if (infile.good()) {
    std::cerr << "Bagfile '" << file_name << "' already exists." << std::endl;
    return std::unique_ptr<WritableStorage>();
  }

  try {
    return std::move(std::make_unique<SqliteStorage>(file_name, true));
  } catch (std::exception & e) {
    std::cerr << "Could not initialize storage. Error: " << e.what() << std::endl;
  }

  return std::unique_ptr<WritableStorage>();
}

}  // namespace rosbag2
