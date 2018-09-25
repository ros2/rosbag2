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

#ifndef ROSBAG2__MOCK_METADATA_IO_HPP_
#define ROSBAG2__MOCK_METADATA_IO_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/metadata_io_iface.hpp"

class MockMetadataIO : public rosbag2_storage::MetadataIOIface
{
public:
  MOCK_METHOD1(write_metadata, void(rosbag2_storage::BagMetadata));
  MOCK_METHOD0(read_metadata, rosbag2_storage::BagMetadata());
};

#endif  // ROSBAG2__MOCK_METADATA_IO_HPP_
