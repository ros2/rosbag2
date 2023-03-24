// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <utility>

#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reader.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT

namespace
{
constexpr const char kStorageId[] = "StoragePlugin";
constexpr const char kRmwFormat[] = "rmw1_format";
}  // namespace

class StorageWithoutMetadataFileTest : public Test
{
public:
  StorageWithoutMetadataFileTest()
  : storage_{std::make_shared<NiceMock<MockStorage>>()},
    converter_factory_{std::make_shared<NiceMock<MockConverterFactory>>()}
  {}

  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<NiceMock<MockConverterFactory>> converter_factory_;
};

TEST_F(StorageWithoutMetadataFileTest, open_uses_storage_id_from_storage_options) {
  {
    auto topic_with_type = rosbag2_storage::TopicMetadata{
      "topic",
      "test_msgs/BasicTypes",
      kRmwFormat,
      "",
    };

    auto topic_information = rosbag2_storage::TopicInformation{
      topic_with_type,
      1
    };

    rosbag2_storage::BagMetadata metadata;
    metadata.relative_file_paths = {"TestPath"};
    metadata.topics_with_message_count = {topic_information};

    EXPECT_CALL(*storage_, get_metadata).Times(1).WillOnce(Return(metadata));
    EXPECT_CALL(*storage_, set_read_order).Times(1).WillOnce(Return(true));
  }

  auto storage_factory = std::make_unique<StrictMock<MockStorageFactory>>();
  EXPECT_CALL(
    *storage_factory,
    open_read_only(_)).Times(1).WillOnce(Return(storage_));

  auto metadata_io = std::make_unique<StrictMock<MockMetadataIo>>();
  EXPECT_CALL(*metadata_io, metadata_file_exists).Times(1).WillOnce(Return(false));

  rosbag2_storage::StorageOptions storage_options;
  storage_options.storage_id = kStorageId;

  auto sequential_reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>(
    std::move(storage_factory),
    converter_factory_,
    std::move(metadata_io));

  sequential_reader->open(storage_options, {"", kRmwFormat});
}
