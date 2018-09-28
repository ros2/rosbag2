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

#ifndef ROSBAG2_STORAGE__METADATA_IO_IFACE_HPP_
#define ROSBAG2_STORAGE__METADATA_IO_IFACE_HPP_

#include <string>

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/visibility_control.hpp"

namespace rosbag2_storage
{

class ROSBAG2_STORAGE_PUBLIC MetadataIoIface
{
public:
  virtual ~MetadataIoIface() = default;

  virtual void write_metadata(const std::string & uri, BagMetadata metadata) = 0;
  virtual BagMetadata read_metadata(const std::string & uri) = 0;
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__METADATA_IO_IFACE_HPP_
