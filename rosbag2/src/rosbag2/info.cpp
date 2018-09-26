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

#include "rosbag2/info.hpp"

#include <memory>
#include <string>
#include <utility>

#include "rosbag2_storage/metadata_io.hpp"

namespace rosbag2
{

Info::Info(std::shared_ptr<rosbag2_storage::StorageFactoryIface> storage_factory)
: storage_factory_(std::move(storage_factory))
{}

rosbag2_storage::BagMetadata Info::read_metadata(const std::string & uri)
{
  return storage_factory_->metadata_io()->read_metadata(uri);
}

}  // namespace rosbag2
