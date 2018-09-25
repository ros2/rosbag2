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

#include "mock_metadata_io.hpp"
#include "rosbag2/info.hpp"
#include "rosbag2_storage/bag_metadata.hpp"
#include "temporary_directory_fixture.hpp"

using namespace ::testing;  // NOLINT

class InfoTestFixture : public TemporaryDirectoryFixture
{
public:
  InfoTestFixture()
  : metadata_io_(std::make_shared<MockMetadataIO>()),
    info_(std::make_shared<rosbag2::Info>(metadata_io_)) {}

  std::shared_ptr<MockMetadataIO> metadata_io_;
  std::shared_ptr<rosbag2::Info> info_;
};

TEST_F(InfoTestFixture, read_metadata_makes_appropriate_call_to_metadata_io_method) {
  rosbag2_storage::BagMetadata metadata;
  EXPECT_CALL(*metadata_io_, read_metadata("test/uri")).WillOnce(Return(metadata));

  info_->read_metadata("test/uri");
}

TEST_F(InfoTestFixture, write_metadata_makes_appropriate_call_to_metadata_io_method) {
  rosbag2_storage::BagMetadata metadata;
  EXPECT_CALL(*metadata_io_, write_metadata("test/uri", _));

  info_->write_metadata("test/uri", metadata);
}
