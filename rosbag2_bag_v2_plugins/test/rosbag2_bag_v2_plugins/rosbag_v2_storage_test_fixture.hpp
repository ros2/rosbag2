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

#ifndef ROSBAG2_BAG_V2_PLUGINS__ROSBAG_V2_STORAGE_TEST_FIXTURE_HPP_
#define ROSBAG2_BAG_V2_PLUGINS__ROSBAG_V2_STORAGE_TEST_FIXTURE_HPP_

#include <memory>
#include <string>

#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/filesystem_helper.hpp"
#include "../../src/rosbag2_bag_v2_plugins/storage/rosbag_v2_storage.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "std_msgs/String.h"

class RosbagV2StorageTestFixture : public testing::Test
{
public:
  RosbagV2StorageTestFixture()
  {
    database_path_ = _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt
    bag_path_ = rosbag2_storage::FilesystemHelper::concat({database_path_, "test_bag.bag"});
    storage_ = std::make_shared<rosbag2_bag_v2_plugins::RosbagV2Storage>();
    storage_->open(bag_path_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
  }

  std::shared_ptr<rosbag2_bag_v2_plugins::RosbagV2Storage> storage_;
  std::string database_path_;
  std::string bag_path_;
};

#endif  // ROSBAG2_BAG_V2_PLUGINS__ROSBAG_V2_STORAGE_TEST_FIXTURE_HPP_
