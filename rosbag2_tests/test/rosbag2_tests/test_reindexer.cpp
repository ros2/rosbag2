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

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"
#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reindexer.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_storage/message_definition.hpp"

#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"

#include "std_msgs/msg/string.hpp"

using namespace testing;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT


class ReindexTestFixture : public ParametrizedTemporaryDirectoryFixture
{
public:
  void SetUp() override
  {
    auto bag_name = get_test_name() + "_" + GetParam();
    root_bag_path_ = rcpputils::fs::path(temporary_dir_path_) / bag_name;
  }

  void TearDown() override
  {
    rcpputils::fs::remove_all(root_bag_path_);
  }

  std::string get_test_name() const
  {
    const auto * test_info = UnitTest::GetInstance()->current_test_info();
    std::string test_name = test_info->name();
    // Replace any slashes in the test name, since it is used in paths
    std::replace(test_name.begin(), test_name.end(), '/', '_');
    return test_name;
  }

  void create_test_bag(int messages_per_file, int num_files)
  {
    {
      rosbag2_cpp::Writer writer;
      rosbag2_storage::StorageOptions storage_options;
      storage_options.storage_id = GetParam();
      storage_options.uri = root_bag_path_.string();
      writer.open(storage_options);
      rosbag2_storage::TopicMetadata topic;
      topic.name = "/test_topic";
      topic.type = "std_msgs/msg/String";
      rosbag2_storage::MessageDefinition md;
      md.name = topic.name;
      writer.create_topic(topic, md);

      std_msgs::msg::String msg;
      rclcpp::Time stamp;
      auto dt = rclcpp::Duration::from_nanoseconds(10 * 1000 * 1000);

      for (int file_i = 0; file_i < num_files; file_i++) {
        for (int msg_i = 0; msg_i < messages_per_file; msg_i++) {
          std::stringstream ss;
          ss << "file" << file_i << "msg" << msg_i;
          msg.data = ss.str();
          writer.write(msg, topic.name, stamp);
          stamp += dt;
        }
        writer.split_bagfile();
      }
    }

    rosbag2_storage::MetadataIo metadata_io;
    original_metadata_ = metadata_io.read_metadata(root_bag_path_.string());
    rcpputils::fs::remove(root_bag_path_ / "metadata.yaml");
  }

  rcpputils::fs::path root_bag_path_;
  rosbag2_storage::BagMetadata original_metadata_;
};

TEST_P(ReindexTestFixture, test_multiple_files) {
  create_test_bag(16, 3);

  rosbag2_storage::MetadataIo metadata_io{};
  rosbag2_cpp::Reindexer reindexer{};


  rosbag2_storage::StorageOptions storage_options{};
  storage_options.uri = root_bag_path_.string();

  ASSERT_FALSE(metadata_io.metadata_file_exists(root_bag_path_.string()));
  reindexer.reindex(storage_options);
  ASSERT_TRUE(metadata_io.metadata_file_exists(root_bag_path_.string()));

  auto generated_metadata = metadata_io.read_metadata(root_bag_path_.string());
  auto target_metadata = original_metadata_;

  EXPECT_EQ(generated_metadata.storage_identifier, GetParam());
  EXPECT_EQ(generated_metadata.version, target_metadata.version);

  for (const auto & gen_rel_path : generated_metadata.relative_file_paths) {
    EXPECT_TRUE(
      std::find(
        target_metadata.relative_file_paths.begin(),
        target_metadata.relative_file_paths.end(),
        gen_rel_path) != target_metadata.relative_file_paths.end());
  }

  EXPECT_EQ(generated_metadata.message_count, target_metadata.message_count);

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
}

INSTANTIATE_TEST_SUITE_P(
  ParametrizedReindexerTests,
  ReindexTestFixture,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
