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

#ifndef ROSBAG2__INFO_HPP_
#define ROSBAG2__INFO_HPP_

#include <memory>
#include <string>

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/metadata_io_iface.hpp"
#include "visibility_control.hpp"

namespace rosbag2
{

class ROSBAG2_PUBLIC Info
{
public:
  Info();
  explicit Info(std::shared_ptr<rosbag2_storage::MetadataIOIface> metadata_io);

  rosbag2_storage::BagMetadata read_metadata(const std::string & uri);
  void write_metadata(const std::string & uri, rosbag2_storage::BagMetadata metadata);

private:
  std::shared_ptr<rosbag2_storage::MetadataIOIface> metadata_io_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__INFO_HPP_
