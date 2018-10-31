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
#include <utility>
#include <vector>

#include "rosbag2/writer.hpp"
#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/topic_with_type.hpp"

#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT

class WriterTest : public Test
{
public:
  WriterTest()
  {
    storage_factory_ = std::make_unique<StrictMock<MockStorageFactory>>();
    storage_ = std::make_shared<NiceMock<MockStorage>>();
    converter_factory_ = std::make_shared<StrictMock<MockConverterFactory>>();
    metadata_io_ = std::make_unique<NiceMock<MockMetadataIo>>();
    storage_options_.uri = "uri";

    EXPECT_CALL(
      *storage_factory_, open_read_write(_, _)).Times(AtLeast(0)).WillRepeatedly(Return(storage_));
  }

  std::unique_ptr<StrictMock<MockStorageFactory>> storage_factory_;
  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<MockMetadataIo> metadata_io_;
  std::unique_ptr<rosbag2::Writer> writer_;
  rosbag2::StorageOptions storage_options_;
};

TEST_F(WriterTest,
  write_uses_converters_to_convert_serialization_format_if_input_and_output_format_are_different) {
  writer_ = std::make_unique<rosbag2::Writer>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));

  std::string storage_serialization_format = "rmw1_format";
  std::string input_format = "rmw2_format";

  auto format1_converter = std::make_unique<StrictMock<MockConverter>>();
  auto format2_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(*format1_converter, serialize(_, _, _)).Times(1);
  EXPECT_CALL(*format2_converter, deserialize(_, _, _)).Times(1);

  EXPECT_CALL(*converter_factory_, load_converter(storage_serialization_format))
  .WillOnce(Return(ByMove(std::move(format1_converter))));
  EXPECT_CALL(*converter_factory_, load_converter(input_format))
  .WillOnce(Return(ByMove(std::move(format2_converter))));

  auto message = std::make_shared<rosbag2::SerializedBagMessage>();
  message->topic_name = "test_topic";
  writer_->open(storage_options_, input_format, storage_serialization_format);
  writer_->create_topic({"test_topic", "test_msgs/Primitives", ""});
  writer_->write(message);
}

TEST_F(WriterTest, write_does_not_use_converters_if_input_and_output_format_are_equal) {
  writer_ = std::make_unique<rosbag2::Writer>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));

  std::string storage_serialization_format = "rmw_format";

  EXPECT_CALL(*converter_factory_, load_converter(storage_serialization_format)).Times(0);

  auto message = std::make_shared<rosbag2::SerializedBagMessage>();
  message->topic_name = "test_topic";
  writer_->open(storage_options_, storage_serialization_format, storage_serialization_format);
  writer_->create_topic({"test_topic", "test_msgs/Primitives", ""});
  writer_->write(message);
}

TEST_F(WriterTest, metadata_io_writes_metadata_file_in_destructor) {
  EXPECT_CALL(*metadata_io_, write_metadata(_, _)).Times(1);
  writer_ = std::make_unique<rosbag2::Writer>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));

  std::string rmw_format = "rmw_format";

  writer_->open(storage_options_, rmw_format, rmw_format);
  writer_.reset();
}

TEST_F(WriterTest, open_throws_error_if_converter_plugin_does_not_exist) {
  writer_ = std::make_unique<rosbag2::Writer>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));

  std::string input_format = "rmw1_format";
  std::string output_format = "rmw2_format";

  auto format1_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(*converter_factory_, load_converter(input_format))
  .WillOnce(Return(ByMove(std::move(format1_converter))));
  EXPECT_CALL(*converter_factory_, load_converter(output_format))
  .WillOnce(Return(ByMove(nullptr)));

  EXPECT_ANY_THROW(writer_->open(storage_options_, input_format, output_format));
}
