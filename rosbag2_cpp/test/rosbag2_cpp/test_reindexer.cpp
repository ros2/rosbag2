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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/asserts.hpp"
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
    target_dir = database_path + "/target_metadata";
  }

  std::string database_path;
  std::string target_dir;
};

TEST_F(ReindexTestFixture, test_multiple_files) {
  auto bag_dir = database_path + "/multiple_files";
  std::unique_ptr<rosbag2_cpp::Reindexer> reindexer =
    std::make_unique<rosbag2_cpp::Reindexer>();

  rosbag2_storage::StorageOptions so = rosbag2_storage::StorageOptions();
  so.uri = bag_dir;
  so.storage_id = "sqlite3";

  reindexer->reindex(so);

  // We should have a metadata file at this point
  auto generated_file = rcpputils::fs::path(bag_dir + "/metadata.yaml");
  EXPECT_TRUE(generated_file.exists());

  // We made it here. Excellent. Truly excellent.
  // Read the generated and target metadata
  auto metadata_io = std::make_unique<rosbag2_storage::MetadataIo>();
  auto generated_metadata = metadata_io->read_metadata(bag_dir);
  auto target_metadata = metadata_io->read_metadata(target_dir + "/multiple_files");

  // Compare versions
  EXPECT_EQ(generated_metadata.version, target_metadata.version);

  // Compare relative file paths
  for (const auto & gen_rel_path : generated_metadata.relative_file_paths) {
    EXPECT_TRUE(
      std::find(
        target_metadata.relative_file_paths.begin(),
        target_metadata.relative_file_paths.end(),
        gen_rel_path) != target_metadata.relative_file_paths.end());
  }

  // Compare starting times
  // Disabled for now, since it may not be possible to 100% recreate
  //   original starting time from metadata
  // EXPECT_EQ(generated_metadata.starting_time, target_metadata.starting_time);

  // Compare durations
  // Disabled for now, since I'm not sure how duraction is created, and if it's correct (jhdcs)
  // EXPECT_EQ(generated_metadata.duration, target_metadata.duration);

  // Compare message count
  EXPECT_EQ(generated_metadata.message_count, target_metadata.message_count);

  // Compare topic information
  // Reindexer can only reconstruct topics that had messages, so not all topics may exist
  for (const auto & gen_topic : generated_metadata.topics_with_message_count) {
    EXPECT_TRUE(
      std::find_if(
        target_metadata.topics_with_message_count.begin(),
        target_metadata.topics_with_message_count.end(),
        [&gen_topic](rosbag2_storage::TopicInformation & t) {
          return (t.topic_metadata.name == gen_topic.topic_metadata.name) &&
          (t.message_count == gen_topic.message_count) &&
          (t.topic_metadata.offered_qos_profiles ==
          gen_topic.topic_metadata.offered_qos_profiles) &&
          (t.topic_metadata.type == gen_topic.topic_metadata.type);
        }
      ) != target_metadata.topics_with_message_count.end()
    );
  }

  // Clean up generated file
  if (generated_file.exists()) {
    remove(generated_file);
  }
}
