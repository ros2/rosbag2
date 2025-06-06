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
#include <rcpputils/filesystem_helper.hpp>

#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT
namespace fs = std::filesystem;

class ReindexerTest : public Test
{
public:
  ReindexerTest()
  : storage_(std::make_shared<NiceMock<MockStorage>>()),
    converter_factory_(std::make_shared<StrictMock<MockConverterFactory>>()),
    storage_serialization_format_("rmw1_format"),
    storage_uri_(rcpputils::fs::create_temporary_directory("test_reindexer").generic_string()),
    relative_path_1_("some_relative_path_1.mcap"),
    relative_path_2_("some_relative_path_2.mcap"),
    relative_path_3_("some_relative_path_3.mcap"),
    default_storage_options_({storage_uri_, ""})
  {}

  virtual void write_file(const std::string & filename)
  {
    std::ofstream outfile;
    outfile.open(fs::path{storage_uri_}.append(filename));
    outfile.close();
  }

  virtual void init()
  {
    // path 1 and path 2 are sequential
    path_1_metadata_.bag_size = 10;
    path_1_metadata_.duration = std::chrono::seconds{10};
    path_1_metadata_.starting_time = std::chrono::system_clock::now();

    path_2_metadata_.bag_size = 15;
    path_2_metadata_.duration = std::chrono::seconds{10};
    path_2_metadata_.starting_time = path_1_metadata_.starting_time + path_1_metadata_.duration;

    // path 3 is parallel
    path_3_metadata_.bag_size = 22;
    path_3_metadata_.duration = std::chrono::seconds{15};
    path_3_metadata_.starting_time = path_1_metadata_.starting_time - std::chrono::seconds{1};
    write_file(relative_path_1_);
    write_file(relative_path_2_);
    write_file(relative_path_3_);

    auto topic_with_type = rosbag2_storage::TopicMetadata{
      0u, "topic", "test_msgs/BasicTypes", storage_serialization_format_, {}, ""};
    auto topics_and_types = std::vector<rosbag2_storage::TopicMetadata>{topic_with_type};
    path_1_metadata_.topics_with_message_count.push_back({topic_with_type, 10});
    path_2_metadata_.topics_with_message_count.push_back({topic_with_type, 10});
    path_3_metadata_.topics_with_message_count.push_back({topic_with_type, 10});

    auto storage_factory = std::make_unique<StrictMock<MockStorageFactory>>();
    auto metadata_io = std::make_unique<NiceMock<MockMetadataIo>>();
    EXPECT_CALL(*metadata_io, write_metadata)
    .WillRepeatedly([this](const std::string &, const rosbag2_storage::BagMetadata & metadata)
      {
        last_bag_metadata = metadata;
    });
    EXPECT_CALL(*metadata_io, metadata_file_exists(_)).WillRepeatedly(Return(true));

    EXPECT_CALL(*storage_, get_all_topics_and_types())
    .Times(AtMost(1)).WillRepeatedly(Return(topics_and_types));
    ON_CALL(*storage_, set_read_order).WillByDefault(Return(true));
    EXPECT_CALL(*storage_, get_metadata()).Times(3)
    .WillOnce(Return (path_1_metadata_))
    .WillOnce(Return(path_2_metadata_))
    .WillOnce(Return(path_3_metadata_));
    EXPECT_CALL(*storage_factory, open_read_only(_)).WillRepeatedly(Return(storage_));

    reindexer_ = std::make_unique<rosbag2_cpp::Reindexer>(std::move(storage_factory),
      std::move(metadata_io));
  }

  ~ReindexerTest() override = default;

  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::string storage_serialization_format_;
  std::string storage_uri_;
  std::string relative_path_1_;
  std::string relative_path_2_;
  std::string relative_path_3_;
  rosbag2_storage::BagMetadata path_1_metadata_;
  rosbag2_storage::BagMetadata path_2_metadata_;
  rosbag2_storage::BagMetadata path_3_metadata_;
  rosbag2_storage::BagMetadata last_bag_metadata;
  rosbag2_storage::StorageOptions default_storage_options_;
  std::unique_ptr<rosbag2_cpp::Reindexer> reindexer_;
};

TEST_F(ReindexerTest, duration_and_start_time_correct)
{
  init();

  rosbag2_storage::StorageOptions storage_options = default_storage_options_;
  reindexer_->reindex(storage_options);
  EXPECT_EQ(last_bag_metadata.starting_time.time_since_epoch().count(),
    path_3_metadata_.starting_time.time_since_epoch().count());
  EXPECT_EQ(last_bag_metadata.duration.count(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds{21}).count());
}
