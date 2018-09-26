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

#ifndef ROSBAG2_STORAGE__METADATA_IO_IMPL_HPP_
#define ROSBAG2_STORAGE__METADATA_IO_IMPL_HPP_

#include <string>

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/topic_with_type.hpp"
#include "rosbag2_storage/visibility_control.hpp"

namespace rosbag2_storage
{

class MetadataIoImpl : public MetadataIo
{
public:
  static constexpr const char * const metadata_filename = "metadata.yaml";

  ROSBAG2_STORAGE_PUBLIC ~MetadataIoImpl() override = default;

  ROSBAG2_STORAGE_PUBLIC void write_metadata(
    const std::string & uri, BagMetadata metadata) override;
  ROSBAG2_STORAGE_PUBLIC BagMetadata read_metadata(const std::string & uri) override;

private:
  std::string get_metadata_file_name(const std::string & uri);
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__METADATA_IO_IMPL_HPP_
