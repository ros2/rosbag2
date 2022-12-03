// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <string>
#include <vector>
#include <utility>

#include "rcpputils/filesystem_helper.hpp"
#include "rosbag2_storage/default_storage_id.hpp"
#include "rosbag2_transport/bag_rewrite.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

using namespace ::testing;  // NOLINT


/*
Builtin knowledge about the bags under test:
rewriter_a:
  - a_empty:
    - test_msgs/Empty
    - 100 messages
    - 2 offered QoS profiles
  - b_basictypes:
    - test_msgs/BasicTypes
    - 50 messages
    - 1 offered QoS profile
rewriter_b:
  - a_empty:
    - test_msgs/Empty
    - 25 messages
    - 1 offered Qos Profile
  - c_strings:
    - test_msgs/Strings
    - 50 messages
    - 1 offered QoS Profile
*/
class TestRewrite : public Test
{
public:
  TestRewrite()
  : output_dir_(rcpputils::fs::create_temp_directory("test_bag_rewrite"))
  {}

  void use_input_a()
  {
    rosbag2_storage::StorageOptions storage;
    storage.uri = (bags_path_ / "rewriter_a").string();
    input_bags_.push_back(storage);
  }

  void use_input_b()
  {
    rosbag2_storage::StorageOptions storage;
    storage.uri = (bags_path_ / "rewriter_b").string();
    input_bags_.push_back(storage);
  }

  ~TestRewrite()
  {
    // rcpputils::fs::remove_all(output_dir_);
  }

  const rcpputils::fs::path bags_path_{_SRC_RESOURCES_DIR_PATH};
  const rcpputils::fs::path output_dir_;
  std::vector<rosbag2_storage::StorageOptions> input_bags_;
  std::vector<std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions>>
  output_bags_;
};

TEST_F(TestRewrite, test_noop_rewrite) {
  use_input_a();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "unchanged").string();
  output_storage.storage_id = rosbag2_storage::get_default_storage_id();
  rosbag2_transport::RecordOptions output_record;
  output_record.all = true;
  output_bags_.push_back({output_storage, output_record});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output_storage);
  reader->open(output_storage);
  const auto metadata = reader->get_metadata();
  EXPECT_EQ(metadata.message_count, 100u + 50u);
  EXPECT_THAT(metadata.topics_with_message_count, SizeIs(2));
  EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.serialization_format, "cdr");
}

TEST_F(TestRewrite, test_merge) {
  use_input_a();
  use_input_b();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "merged").string();
  output_storage.storage_id = rosbag2_storage::get_default_storage_id();
  rosbag2_transport::RecordOptions output_record;
  output_record.all = true;
  output_bags_.push_back({output_storage, output_record});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output_storage);
  reader->open(output_storage);
  const auto metadata = reader->get_metadata();
  EXPECT_EQ(metadata.message_count, 100u + 50u + 50u + 25u);

  // Check that all 3 topics are present (a_empty merged from both)
  EXPECT_THAT(metadata.topics_with_message_count, SizeIs(3));

  // Check that offered QoS profiles got concatenated
  for (const auto & topic_info : metadata.topics_with_message_count) {
    const auto topic = topic_info.topic_metadata;
    if (topic.name == "a_empty") {
      YAML::Node qos_node = YAML::Load(topic.offered_qos_profiles);
      EXPECT_TRUE(qos_node.IsSequence());
      EXPECT_EQ(qos_node.size(), 3u);
    }
  }
}

TEST_F(TestRewrite, test_filter_split) {
  use_input_a();

  {
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = (output_dir_ / "split1").string();
    storage_opts.storage_id = rosbag2_storage::get_default_storage_id();
    rosbag2_transport::RecordOptions rec_opts;
    rec_opts.all = true;
    rec_opts.exclude = "basic";
    output_bags_.push_back({storage_opts, rec_opts});
  }
  {
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = (output_dir_ / "split2").string();
    storage_opts.storage_id = rosbag2_storage::get_default_storage_id();
    rosbag2_transport::RecordOptions rec_opts;
    rec_opts.all = false;
    rec_opts.topics = {"b_basictypes"};
    output_bags_.push_back({storage_opts, rec_opts});
  }

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  {
    auto opts = output_bags_[0].first;
    auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(opts);
    reader->open(opts);
    const auto metadata = reader->get_metadata();
    EXPECT_THAT(metadata.topics_with_message_count, SizeIs(1));
    EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.name, "a_empty");
    EXPECT_EQ(metadata.message_count, 100u);
  }
  {
    auto opts = output_bags_[1].first;
    auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(opts);
    reader->open(opts);
    const auto metadata = reader->get_metadata();
    EXPECT_THAT(metadata.topics_with_message_count, SizeIs(1));
    EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.name, "b_basictypes");
    EXPECT_EQ(metadata.message_count, 50u);
  }
}

TEST_F(TestRewrite, test_compress) {
  use_input_a();

  rosbag2_storage::StorageOptions output_storage;
  auto out_bag = output_dir_ / "compressed";
  output_storage.uri = out_bag.string();
  output_storage.storage_id = rosbag2_storage::get_default_storage_id();
  rosbag2_transport::RecordOptions output_record;
  output_record.all = true;
  output_record.compression_mode = "file";
  output_record.compression_format = "zstd";
  output_bags_.push_back({output_storage, output_record});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  rosbag2_storage::MetadataIo metadata_io;
  auto metadata = metadata_io.read_metadata(out_bag.string());
  auto first_storage = out_bag / metadata.relative_file_paths[0];

  EXPECT_EQ(first_storage.extension().string(), ".zstd");
  EXPECT_TRUE(first_storage.exists());
  EXPECT_TRUE(first_storage.is_regular_file());
}
