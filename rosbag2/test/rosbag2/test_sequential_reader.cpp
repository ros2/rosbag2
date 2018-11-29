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

#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT

class SequentialReaderTest : public Test
{
public:
  SequentialReaderTest()
  {
    storage_factory_ = std::make_unique<StrictMock<MockStorageFactory>>();
    storage_ = std::make_shared<NiceMock<MockStorage>>();
    converter_factory_ = std::make_shared<StrictMock<MockConverterFactory>>();

    rosbag2_storage::TopicMetadata topic_with_type;
    topic_with_type.name = "topic";
    topic_with_type.type = "test_msgs/Primitives";
    auto topics_and_types = std::vector<rosbag2_storage::TopicMetadata>{topic_with_type};
    EXPECT_CALL(*storage_, get_all_topics_and_types())
    .Times(AtMost(1)).WillRepeatedly(Return(topics_and_types));

    auto message = std::make_shared<rosbag2::SerializedBagMessage>();
    message->topic_name = topic_with_type.name;
    EXPECT_CALL(*storage_, read_next()).WillRepeatedly(Return(message));

    EXPECT_CALL(*storage_factory_, open_read_only(_, _)).WillOnce(Return(storage_));

    reader_ = std::make_unique<rosbag2::SequentialReader>(
      std::move(storage_factory_), converter_factory_);
  }

  void set_storage_serialization_format(const std::string & format)
  {
    rosbag2_storage::BagMetadata metadata;
    metadata.topics_with_message_count.push_back({{"", "", format}, 1});
    EXPECT_CALL(*storage_, get_metadata()).WillOnce(Return(metadata));
  }

  std::unique_ptr<StrictMock<MockStorageFactory>> storage_factory_;
  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<rosbag2::SequentialReader> reader_;
};

TEST_F(SequentialReaderTest, read_next_uses_converters_to_convert_serialization_format) {
  std::string storage_serialization_format = "rmw1_format";
  std::string output_format = "rmw2_format";
  set_storage_serialization_format(storage_serialization_format);

  auto format1_converter = std::make_unique<StrictMock<MockConverter>>();
  auto format2_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(*format1_converter, deserialize(_, _, _)).Times(1);
  EXPECT_CALL(*format2_converter, serialize(_, _, _)).Times(1);

  EXPECT_CALL(*converter_factory_, load_converter(storage_serialization_format))
  .WillOnce(Return(ByMove(std::move(format1_converter))));
  EXPECT_CALL(*converter_factory_, load_converter(output_format))
  .WillOnce(Return(ByMove(std::move(format2_converter))));

  reader_->open(rosbag2::StorageOptions(), {"", output_format});
  reader_->read_next();
}

TEST_F(SequentialReaderTest, open_throws_error_if_converter_plugin_does_not_exist) {
  std::string storage_serialization_format = "rmw1_format";
  std::string output_format = "rmw2_format";
  set_storage_serialization_format(storage_serialization_format);

  auto format1_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(*converter_factory_, load_converter(storage_serialization_format))
  .WillOnce(Return(ByMove(std::move(format1_converter))));
  EXPECT_CALL(*converter_factory_, load_converter(output_format))
  .WillOnce(Return(ByMove(nullptr)));

  EXPECT_ANY_THROW(reader_->open(rosbag2::StorageOptions(), {"", output_format}));
}

TEST_F(SequentialReaderTest,
  read_next_does_not_use_converters_if_input_and_output_format_are_equal)
{
  std::string storage_serialization_format = "rmw1_format";
  set_storage_serialization_format(storage_serialization_format);

  EXPECT_CALL(*converter_factory_, load_converter(storage_serialization_format)).Times(0);

  reader_->open(rosbag2::StorageOptions(), {"", storage_serialization_format});
  reader_->read_next();
}
