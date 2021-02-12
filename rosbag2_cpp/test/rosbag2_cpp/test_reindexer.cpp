// Copyright 2021 DCS Corporation, All Rights Reserved.
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
//
// DISTRIBUTION A. Approved for public release; distribution unlimited.
// OPSEC #4584.
//
// Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS
// Part 252.227-7013 or 7014 (Feb 2014).
//
// This notice must appear in all copies of this file and its derivatives.

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reindexer.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT

class ReindexTestFixture : public Test
{
public:
  ReindexTestFixture()
  {
    database_path = _SRC_REINDEX_DIR_PATH;
  }

  // std::shared_ptr<NiceMock<MockStorage>> storage_;
  // std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  // std::unique_ptr<rosbag2_cpp::Reader> reader_;
  // std::string storage_serialization_format_;
  std::string database_path;
  // rosbag2_storage::StorageOptions default_storage_options_;
};

TEST_F(ReindexTestFixture, test_multiple_files) {
  std::unique_ptr<rosbag2_cpp::reindexers::Reindexer> reindexer =
    std::make_unique<rosbag2_cpp::reindexers::Reindexer>();

  rosbag2_storage::StorageOptions so = rosbag2_storage::StorageOptions();
  so.uri = database_path + "/multiple_files";
  so.storage_id = "sqlite3";

  reindexer->reindex(so);
}
