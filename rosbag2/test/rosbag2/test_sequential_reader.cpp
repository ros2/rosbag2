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

#include "rosbag2/sequential_reader.hpp"
#include "rosbag2_storage/bag_metadata.hpp"

#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT
using rosbag2::SequentialReader;

class SequentialReaderTest : public Test
{
};

TEST_F(SequentialReaderTest, read_next_uses_converters_to_convert_serialization_format) {
  auto storage_factory = std::make_unique<StrictMock<MockStorageFactory>>();
  auto storage = std::make_shared<StrictMock<MockStorage>>();
  auto converter_factory = std::make_shared<StrictMock<MockConverterFactory>>();
  auto format1_converter = std::make_shared<StrictMock<MockConverter>>();
  auto format2_converter = std::make_shared<StrictMock<MockConverter>>();

  std::string storage_serialization_format = "rmw1_format";
  std::string output_format = "rmw2_format";
  rosbag2_storage::BagMetadata metadata;
  metadata.storage_format = storage_serialization_format;
  EXPECT_CALL(*storage, get_metadata()).WillOnce(Return(metadata));

  EXPECT_CALL(*storage_factory, open_read_only(_, _)).WillOnce(Return(storage));
  EXPECT_CALL(*converter_factory, load_converter(storage_serialization_format))
  .WillOnce(Return(format1_converter));
  EXPECT_CALL(*converter_factory, load_converter(output_format))
  .WillOnce(Return(format2_converter));

  SequentialReader reader{std::move(storage_factory), converter_factory};
  reader.open(rosbag2::StorageOptions(), output_format);

  EXPECT_CALL(*format1_converter, deserialize(_, _, _)).Times(1);
  EXPECT_CALL(*format2_converter, serialize(_, _, _)).Times(1);
  reader.read_next();
}
