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


#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "rosbag2_storage/filesystem_helper.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag_v2_storage_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

namespace rosbag2_storage
{

bool operator==(const TopicMetadata & lhs, const TopicMetadata & rhs)
{
  return lhs.name == rhs.name && lhs.type == rhs.type;
}

bool operator!=(const TopicMetadata & lhs, const TopicMetadata & rhs)
{
  return !(lhs == rhs);
}

bool operator==(const TopicInformation & lhs, const TopicInformation & rhs)
{
  return lhs.topic_metadata == rhs.topic_metadata &&
         lhs.message_count == rhs.message_count;
}

bool operator!=(const TopicInformation & lhs, const TopicInformation & rhs)
{
  return !(lhs == rhs);
}

}  // namespace rosbag2_storage

TEST_F(RosbagV2StorageTestFixture, get_all_topics_and_types_returns_list_of_recorded_bag_file) {
  std::vector<rosbag2_storage::TopicMetadata> expected_topic_metadata = {
    {"/rosout", "rosgraph_msgs/Log", ""},
    {"/test_topic", "std_msgs/String", ""},
    {"/test_topic2", "std_msgs/String", ""},
  };

  auto topic_metadata = storage_->get_all_topics_and_types();

  for (size_t i = 0; i < expected_topic_metadata.size(); ++i) {
    EXPECT_THAT(topic_metadata[i], expected_topic_metadata[i]);
  }
}

TEST_F(RosbagV2StorageTestFixture, get_metadata_returns_bagfile_description)
{
  std::vector<rosbag2_storage::TopicInformation> expected_topics_with_message_count = {
    {{"/rosout", "rosgraph_msgs/Log", ""}, 3},
    {{"/test_topic", "std_msgs/String", ""}, 1},
    {{"/test_topic2", "std_msgs/String", ""}, 1}
  };

  auto bag_metadata = storage_->get_metadata();

  EXPECT_THAT(bag_metadata.version, Eq(1));
  EXPECT_THAT(bag_metadata.storage_identifier, StrEq("rosbag_v2"));
  EXPECT_THAT(bag_metadata.bag_size, Eq(9023u));
  EXPECT_THAT(bag_metadata.relative_file_paths, ElementsAre("test_bag.bag"));
  EXPECT_THAT(bag_metadata.starting_time,
    Eq(std::chrono::time_point<std::chrono::high_resolution_clock>(1543509813298505673ns)));
  EXPECT_THAT(bag_metadata.duration, Eq(268533408ns));
  EXPECT_THAT(bag_metadata.message_count, Eq(5u));
  EXPECT_THAT(
    bag_metadata.topics_with_message_count, SizeIs(expected_topics_with_message_count.size()));
  for (size_t i = 0; i < expected_topics_with_message_count.size(); ++i) {
    EXPECT_THAT(
      bag_metadata.topics_with_message_count[i], Eq(expected_topics_with_message_count[i]));
  }
}

TEST_F(RosbagV2StorageTestFixture, has_next_only_counts_messages_with_ros2_counterpart)
{
  // There are only two messages that can be read
  EXPECT_TRUE(storage_->has_next());
  storage_->read_next();
  EXPECT_TRUE(storage_->has_next());
  storage_->read_next();
  EXPECT_FALSE(storage_->has_next());
  EXPECT_FALSE(storage_->has_next());  // Once false, it stays false
}

TEST_F(RosbagV2StorageTestFixture, read_next_will_not_read_messages_without_ros2_equivalent)
{
  EXPECT_TRUE(storage_->has_next());
  auto first_message = storage_->read_next();

  EXPECT_TRUE(storage_->has_next());
  auto second_message = storage_->read_next();

  EXPECT_THAT(first_message->topic_name, StrEq("/test_topic"));
  EXPECT_THAT(second_message->topic_name, StrEq("/test_topic2"));
}

TEST_F(RosbagV2StorageTestFixture, read_next_will_produce_messages_ordered_by_timestamp)
{
  EXPECT_TRUE(storage_->has_next());
  auto first_message = storage_->read_next();


  EXPECT_TRUE(storage_->has_next());
  auto second_message = storage_->read_next();

  EXPECT_THAT(second_message->time_stamp, Ge(first_message->time_stamp));
}
