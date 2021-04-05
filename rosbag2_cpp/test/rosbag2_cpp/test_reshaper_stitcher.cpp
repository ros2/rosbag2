// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// Copyright 2021 Firefly Automatix Inc.,
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

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_cpp/reshapers/stitcher.hpp"

#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT

class StitcherTest : public Test
{
public:
  StitcherTest()
  : storage_(std::make_shared<NiceMock<MockStorage>>()),
    converter_factory_(std::make_shared<StrictMock<MockConverterFactory>>()),
    storage_serialization_format_("rmw1_format"),
    output_uri_(rcpputils::fs::temp_directory_path().string()),
    relative_path_1_("some_relative_path_1"),
    relative_path_2_("some_relative_path_2"),
    relative_path_3_("some_relative_path_3"),
    absolute_path_1_((rcpputils::fs::path(output_uri_) / "some/folder").string()),
    default_storage_options_({output_uri_, ""})
  {}

  virtual void init()
  {
    auto storage_factory = std::make_unique<StrictMock<MockStorageFactory>>();
    auto metadata_io = std::make_unique<NiceMock<MockMetadataIo>>();

    stitcher_ = std::make_unique<rosbag2_cpp::Stitcher>();
  }

  virtual ~StitcherTest() = default;

  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<rosbag2_cpp::Stitcher> stitcher_;
  std::string storage_serialization_format_;
  std::string output_uri_;
  std::string relative_path_1_;
  std::string relative_path_2_;
  std::string relative_path_3_;
  std::string absolute_path_1_;
  rosbag2_storage::StorageOptions default_storage_options_;
};

TEST_F(StitcherTest, has_next_throws_if_no_storage)
{
  init();

  EXPECT_ANY_THROW(stitcher_->has_next());
}

TEST_F(StitcherTest, stitch_next_throws_if_no_storage)
{
  init();

  EXPECT_ANY_THROW(stitcher_->stitch_next());
}
