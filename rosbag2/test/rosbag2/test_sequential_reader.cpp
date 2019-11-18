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

#include "rosbag2/readers/sequential_reader.hpp"
#include "rosbag2/reader.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT

class SequentialReaderTest : public Test
{
public:
  SequentialReaderTest()
  : storage_(std::make_shared<NiceMock<MockStorage>>()),
    converter_factory_(std::make_shared<StrictMock<MockConverterFactory>>()),
    storage_serialization_format_("rmw1_format")
  {
    rosbag2_storage::TopicMetadata topic_with_type;
    topic_with_type.name = "topic";
    topic_with_type.type = "test_msgs/BasicTypes";
    topic_with_type.serialization_format = storage_serialization_format_;
    auto topics_and_types = std::vector<rosbag2_storage::TopicMetadata>{topic_with_type};

    auto message = std::make_shared<rosbag2::SerializedBagMessage>();
    message->topic_name = topic_with_type.name;

    auto storage_factory = std::make_unique<StrictMock<MockStorageFactory>>();
    auto metadata_io = std::make_unique<NiceMock<MockMetadataIo>>();
    rosbag2_storage::BagMetadata metadata;
    metadata.relative_file_paths = {"/path/to/storage"};
    metadata.topics_with_message_count.push_back({{topic_with_type}, 1});
    EXPECT_CALL(*metadata_io, read_metadata(_)).WillRepeatedly(Return(metadata));

    EXPECT_CALL(*storage_, get_all_topics_and_types())
    .Times(AtMost(1)).WillRepeatedly(Return(topics_and_types));
    EXPECT_CALL(*storage_, read_next()).WillRepeatedly(Return(message));
    EXPECT_CALL(*storage_factory, open_read_only(_, _)).WillOnce(Return(storage_));

    auto sequential_reader = std::make_unique<rosbag2::readers::SequentialReader>(
      std::move(storage_factory), converter_factory_, std::move(metadata_io));

    reader_ = std::make_unique<rosbag2::Reader>(std::move(sequential_reader));
  }

  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<rosbag2::Reader> reader_;
  std::string storage_serialization_format_;
};

TEST_F(SequentialReaderTest, read_next_uses_converters_to_convert_serialization_format) {
  std::string output_format = "rmw2_format";

  auto format1_converter = std::make_unique<StrictMock<MockConverter>>();
  auto format2_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(*format1_converter, deserialize(_, _, _)).Times(1);
  EXPECT_CALL(*format2_converter, serialize(_, _, _)).Times(1);

  EXPECT_CALL(*converter_factory_, load_deserializer(storage_serialization_format_))
  .WillOnce(Return(ByMove(std::move(format1_converter))));
  EXPECT_CALL(*converter_factory_, load_serializer(output_format))
  .WillOnce(Return(ByMove(std::move(format2_converter))));

  reader_->open(rosbag2::StorageOptions(), {"", output_format});
  reader_->read_next();
}

TEST_F(SequentialReaderTest, open_throws_error_if_converter_plugin_does_not_exist) {
  std::string output_format = "rmw2_format";

  auto format1_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(*converter_factory_, load_deserializer(storage_serialization_format_))
  .WillOnce(Return(ByMove(std::move(format1_converter))));
  EXPECT_CALL(*converter_factory_, load_serializer(output_format))
  .WillOnce(Return(ByMove(nullptr)));

  EXPECT_ANY_THROW(reader_->open(rosbag2::StorageOptions(), {"", output_format}));
}

TEST_F(SequentialReaderTest,
  read_next_does_not_use_converters_if_input_and_output_format_are_equal) {
  std::string storage_serialization_format = "rmw1_format";

  EXPECT_CALL(*converter_factory_, load_deserializer(storage_serialization_format)).Times(0);
  EXPECT_CALL(*converter_factory_, load_serializer(storage_serialization_format)).Times(0);

  reader_->open(rosbag2::StorageOptions(), {"", storage_serialization_format});
  reader_->read_next();
}
