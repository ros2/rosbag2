// Copyright 2019 Epiroc or its affiliates. All Rights Reserved.
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


#include <gmock/gmock.h>

#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <chrono>
#include <fstream>

#include "rosbag2_cpp/reindexer.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"

#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT
namespace fs = std::filesystem;

class ReindexerTest : public rosbag2_test_common::TemporaryDirectoryFixture
{
public:
  ReindexerTest()
  {
    storage_ = std::make_shared<NiceMock<MockStorage>>();
    storage_uri_ = temporary_dir_path_;

    // bag 1 and bag 2 are sequential
    bag_1_metadata_.bag_size = 10;
    bag_1_metadata_.duration = std::chrono::seconds{10};
    bag_1_metadata_.starting_time = std::chrono::high_resolution_clock::now();

    bag_2_metadata_.bag_size = 15;
    bag_2_metadata_.duration = std::chrono::seconds{10};
    bag_2_metadata_.starting_time = bag_1_metadata_.starting_time + bag_1_metadata_.duration;

    // bag 3 overlap with bag 1
    bag_3_metadata_.bag_size = 22;
    bag_3_metadata_.duration = std::chrono::seconds{15};
    bag_3_metadata_.starting_time = bag_1_metadata_.starting_time - std::chrono::seconds{1};
    // We mock storage and meatdata_io. However, the reindexer checking that the storage directory
    // is not empty, so we need to write some files to the storage directory.
    write_file(bag_1_filename);
    write_file(bag_2_filename);
    write_file(bag_3_filename);

    auto topic_with_type = rosbag2_storage::TopicMetadata{
      0u, "topic", "test_msgs/BasicTypes", storage_serialization_format_, {}, ""};
    auto topics_and_types = std::vector<rosbag2_storage::TopicMetadata>{topic_with_type};
    bag_1_metadata_.topics_with_message_count.push_back({topic_with_type, 10});
    bag_2_metadata_.topics_with_message_count.push_back({topic_with_type, 10});
    bag_3_metadata_.topics_with_message_count.push_back({topic_with_type, 10});

    auto storage_factory = std::make_unique<StrictMock<MockStorageFactory>>();
    auto metadata_io = std::make_unique<NiceMock<MockMetadataIo>>();
    EXPECT_CALL(*metadata_io, write_metadata)
    .WillRepeatedly([this](const std::string &, const rosbag2_storage::BagMetadata & metadata)
      {
        saved_metadata_.push_back(metadata);
      });
    EXPECT_CALL(*metadata_io, metadata_file_exists(_)).WillRepeatedly(Return(true));
    ON_CALL(*storage_, set_read_order).WillByDefault(Return(true));
    EXPECT_CALL(*storage_factory, open_read_only(_)).WillRepeatedly(Return(storage_));

    EXPECT_CALL(*storage_, get_all_topics_and_types())
    .Times(AtMost(1)).WillRepeatedly(Return(topics_and_types));

    EXPECT_CALL(*storage_, get_metadata()).Times(3)
    .WillOnce(Return(bag_1_metadata_))
    .WillOnce(Return(bag_2_metadata_))
    .WillOnce(Return(bag_3_metadata_));

    reindexer_ = std::make_unique<rosbag2_cpp::Reindexer>(std::move(storage_factory),
                                                          std::move(metadata_io));
  }

  void write_file(const std::string & filename) const
  {
    std::ofstream outfile;
    outfile.open(fs::path{storage_uri_}.append(filename));
    outfile.close();
  }

  ~ReindexerTest() override = default;

  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::string storage_serialization_format_ = "cdr";
  std::string storage_uri_;
  std::string bag_1_filename = "bag_1.mcap";
  std::string bag_2_filename = "bag_2.mcap";
  std::string bag_3_filename = "bag_3.mcap";
  rosbag2_storage::BagMetadata bag_1_metadata_;
  rosbag2_storage::BagMetadata bag_2_metadata_;
  rosbag2_storage::BagMetadata bag_3_metadata_;
  std::vector<rosbag2_storage::BagMetadata> saved_metadata_;
  std::unique_ptr<rosbag2_cpp::Reindexer> reindexer_;
};

TEST_F(ReindexerTest, duration_and_start_time_correct)
{
  rosbag2_storage::StorageOptions storage_options{storage_uri_, ""};
  reindexer_->reindex(storage_options);
  ASSERT_EQ(saved_metadata_.size(), 1u);
  EXPECT_EQ(saved_metadata_.back().starting_time.time_since_epoch().count(),
            bag_3_metadata_.starting_time.time_since_epoch().count());
  EXPECT_EQ(saved_metadata_.back().duration.count(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds{21}).count());
}
