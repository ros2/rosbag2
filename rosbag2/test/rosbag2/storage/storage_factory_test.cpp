/*
 *  Copyright (c) 2018,  Bosch Software Innovations GmbH.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <fstream>

#include "../../../src/rosbag2/storage/storage_factory.hpp"

#include "../rosbag2_test_fixture.cpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT

class StorageFactoryFixture : public Rosbag2TestFixture
{
public:
  StorageFactoryFixture()
  : factory_(std::make_unique<StorageFactory>()) {}

  std::unique_ptr<StorageFactory> factory_;
};


TEST_F(StorageFactoryFixture, returns_nullptr_if_database_already_exists) {
  std::ofstream file {database_name_};

  auto storage = factory_->get_for_writing(database_name_);

  EXPECT_THAT(storage, IsNull());
}
