// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT

class MultifileReaderTest : public Test
{
public:
  MultifileReaderTest()
  : storage_(std::make_shared<NiceMock<MockStorage>>()),
    converter_factory_(std::make_shared<StrictMock<MockConverterFactory>>()),
    storage_serialization_format_("rmw1_format"),
    relative_path_1_("some_relative_path_1"),
    relative_path_2_("some_relative_path_2")
  {
    auto topic_with_type = rosbag2_storage::TopicMetadata{
      "topic", "test_msgs/BasicTypes", storage_serialization_format_, ""};
    auto topics_and_types = std::vector<rosbag2_storage::TopicMetadata>{topic_with_type};

    auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    message->topic_name = topic_with_type.name;

    auto metadata_io = std::make_unique<NiceMock<MockMetadataIo>>();
    rosbag2_storage::BagMetadata metadata;
    metadata.relative_file_paths.push_back(relative_path_1_);
    metadata.relative_file_paths.push_back(relative_path_2_);
    metadata.topics_with_message_count.push_back({topic_with_type, 10});
    ON_CALL(*metadata_io, read_metadata(_)).WillByDefault(Return(metadata));
    EXPECT_CALL(*metadata_io, metadata_file_exists(_)).WillRepeatedly(Return(true));

    auto storage_factory = std::make_unique<StrictMock<MockStorageFactory>>();
    EXPECT_CALL(*storage_, get_all_topics_and_types())
    .Times(AtMost(1)).WillRepeatedly(Return(topics_and_types));
    ON_CALL(*storage_, read_next()).WillByDefault(Return(message));
    EXPECT_CALL(*storage_factory, open_read_only(_, _)).WillRepeatedly(Return(storage_));

    auto sequential_reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>(
      std::move(storage_factory), converter_factory_, std::move(metadata_io));

    reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
  }

  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::string storage_serialization_format_;
  std::string relative_path_1_;
  std::string relative_path_2_;
};

TEST_F(MultifileReaderTest, has_next_reads_next_file)
{
  // storage::has_next() is called twice when reader::has_next() is called
  EXPECT_CALL(*storage_, has_next()).Times(4)
  .WillOnce(Return(true)).WillOnce(Return(true))  // We have a message
  .WillOnce(Return(false))  // No message, load next file
  .WillOnce(Return(true));  // True since we now have a message
  EXPECT_CALL(*converter_factory_, load_deserializer(storage_serialization_format_)).Times(0);
  EXPECT_CALL(*converter_factory_, load_serializer(storage_serialization_format_)).Times(0);
  reader_->open(rosbag2_cpp::StorageOptions(), {"", storage_serialization_format_});

  auto & sr = static_cast<rosbag2_cpp::readers::SequentialReader &>(
    reader_->get_implementation_handle());

  EXPECT_EQ(sr.get_current_file(), relative_path_1_);
  reader_->has_next();
  reader_->read_next();
  reader_->has_next();
  EXPECT_EQ(sr.get_current_file(), relative_path_2_);
}

TEST_F(MultifileReaderTest, has_next_throws_if_no_storage)
{
  EXPECT_ANY_THROW(reader_->has_next());
}

TEST_F(MultifileReaderTest, read_next_throws_if_no_storage)
{
  EXPECT_ANY_THROW(reader_->read_next());
}

TEST_F(MultifileReaderTest, get_all_topics_and_types_throws_if_no_storage)
{
  EXPECT_ANY_THROW(reader_->get_all_topics_and_types());
}
