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

#include "rosbag2_storage/metadata_io.hpp"

namespace rosbag2
{

Info::Info()
{
  metadata_io_ = std::make_shared<rosbag2_storage::MetadataIO>();
}

Info::Info(std::shared_ptr<rosbag2_storage::MetadataIOIface> metadata_io)
: metadata_io_(metadata_io)
{}

rosbag2_storage::BagMetadata Info::read_metadata(const std::string & uri)
{
  return metadata_io_->read_metadata(uri);
}

void Info::write_metadata(const std::string & uri, rosbag2_storage::BagMetadata metadata)
{
  metadata_io_->write_metadata(uri, metadata);
}

}  // namespace rosbag2
