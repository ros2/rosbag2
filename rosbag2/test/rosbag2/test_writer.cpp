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

#include "rosbag2_storage/filesystem_helpers.hpp"
#include "rosbag2/storage_options.hpp"
#include "../../src/rosbag2/writer_impl.hpp"
#include "mock_storage_factory.hpp"
#include "mock_metadata_io.hpp"
#include "temporary_directory_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

TEST(WriterTests, writer_writes_stored_metadata_on_shutdown) {
  auto metadata_io = std::make_shared<MockMetadataIO>();
  auto storage = std::make_shared<MockStorageFactory>(metadata_io);

  EXPECT_CALL(*metadata_io, write_metadata(_, _));

  rosbag2::StorageOptions options{};
  auto writer = std::make_shared<rosbag2::WriterImpl>(options, storage);
  writer.reset();  // this should call write_metadata
}
