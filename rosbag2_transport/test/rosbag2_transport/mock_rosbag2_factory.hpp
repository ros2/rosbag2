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

#include <memory>

#include "rosbag2/rosbag2_factory.hpp"
#include "rosbag2/info.hpp"
#include "mock_sequential_reader.hpp"
#include "mock_writer.hpp"
#include "mock_info.hpp"

#ifndef ROSBAG2_TRANSPORT__MOCK_ROSBAG2_FACTORY_HPP_
#define ROSBAG2_TRANSPORT__MOCK_ROSBAG2_FACTORY_HPP_

class MockRosbag2Factory : public rosbag2::Rosbag2Factory
{
public:
  MockRosbag2Factory()
  : writer_(std::make_shared<MockWriter>()),
    reader_(std::make_shared<MockSequentialReader>()),
    info_(std::make_shared<MockInfo>())
  {}

  std::shared_ptr<rosbag2::Writer> create_writer(
    const rosbag2::StorageOptions & options) override
  {
    (void) options;
    return writer_;
  }

  std::shared_ptr<rosbag2::SequentialReader> create_sequential_reader(
    const rosbag2::StorageOptions & options) override
  {
    (void) options;
    return reader_;
  }

  std::shared_ptr<rosbag2::Info> create_info() override
  {
    return info_;
  }

  std::shared_ptr<MockWriter> writer_;
  std::shared_ptr<MockSequentialReader> reader_;
  std::shared_ptr<MockInfo> info_;
};

#endif  // ROSBAG2_TRANSPORT__MOCK_ROSBAG2_FACTORY_HPP_
