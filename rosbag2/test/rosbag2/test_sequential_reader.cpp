// Copyright 2018 Open Source Robotics Foundation, Inc.
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
#include <utility>
#include <vector>

#include "rosbag2/sequential_reader.hpp"
#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "mock_converter_factory.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT

class SequentialReaderTest : public Test
{
public:
  SequentialReaderTest()
  {
    storage_factory_ = std::make_unique<NiceMock<MockStorageFactory>>();
    storage_ = std::make_shared<NiceMock<MockStorage>>();
    converter_factory_ = std::make_shared<StrictMock<MockConverterFactory>>();

    rosbag2_storage::TopicMetadata topic_with_type;
    topic_with_type.name = "topic";
    topic_with_type.type = "test_msgs/BasicTypes";
    auto topics_and_types = std::vector<rosbag2_storage::TopicMetadata>{topic_with_type};
    EXPECT_CALL(*storage_, get_all_topics_and_types())
    .Times(AtMost(1)).WillRepeatedly(Return(topics_and_types));

    auto message = std::make_shared<rosbag2::SerializedBagMessage>();
    message->topic_name = topic_with_type.name;
    ON_CALL(*storage_, read_next()).WillByDefault(Return(message));
    ON_CALL(*storage_factory_, open_read_only(_, _)).WillByDefault(Return(storage_));

    // Version 2 of the metadata file includes the split bag features.
    metadata_.version = 2;
    metadata_.storage_identifier = "sqlite3";
    metadata_.relative_file_paths.emplace_back("some_relative_path");
    metadata_.relative_file_paths.emplace_back("some_other_relative_path");
    metadata_.duration = std::chrono::nanoseconds(100);
    metadata_.starting_time =
      std::chrono::time_point<std::chrono::high_resolution_clock>(
      std::chrono::nanoseconds(1000000));
    metadata_.message_count = 50;
    metadata_.topics_with_message_count.push_back({{"topic1", "type1", serialization_format}, 100});
    ON_CALL(*storage_, get_metadata()).WillByDefault(Return(metadata_));

    reader_ = std::make_unique<rosbag2::SequentialReader>(
      std::move(storage_factory_), converter_factory_);
  }

  std::unique_ptr<NiceMock<MockStorageFactory>> storage_factory_;
  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  rosbag2_storage::BagMetadata metadata_;
  std::unique_ptr<rosbag2::SequentialReader> reader_;
  std::string serialization_format {"rmw1_format"};
};

TEST_F(SequentialReaderTest, has_next_reads_next_file)
{
  EXPECT_CALL(*storage_, has_next()).Times(AtLeast(2)).WillRepeatedly(Return(false));
  reader_->open(rosbag2::StorageOptions(), {serialization_format, serialization_format});
  reader_->has_next();
  reader_->read_next();
}

TEST_F(SequentialReaderTest, has_next_throws_if_no_storage)
{
  EXPECT_ANY_THROW(reader_->has_next());
}

TEST_F(SequentialReaderTest, read_next_throws_if_no_storage)
{
  EXPECT_ANY_THROW(reader_->read_next());
}

TEST_F(SequentialReaderTest, get_all_topics_and_types_throws_if_no_storage)
{
  EXPECT_ANY_THROW(reader_->get_all_topics_and_types());
}
