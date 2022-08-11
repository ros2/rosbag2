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

#include "rosbag2_cpp/info.hpp"

#include <memory>
#include <stdexcept>
#include <string>

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_factory.hpp"

namespace rosbag2_cpp
{

rosbag2_storage::BagMetadata Info::read_metadata(
  const std::string & uri, const std::string & storage_id)
{
  rosbag2_storage::MetadataIo metadata_io;
  if (metadata_io.metadata_file_exists(uri)) {
    return metadata_io.read_metadata(uri);
  }

  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_only({uri, storage_id});
  if (!storage) {
    throw std::runtime_error(
            "The metadata.yaml file does not exist and the bag could not be opened.");
  }
  return storage->get_metadata();
}

}  // namespace rosbag2_cpp
