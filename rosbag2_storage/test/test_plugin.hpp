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

#ifndef TEST_PLUGIN_HPP_
#define TEST_PLUGIN_HPP_

#include <string>

#include "rosbag2_storage/writable_storage.hpp"
#include "rosbag2_storage/readable_storage.hpp"

class TestPlugin : public rosbag2_storage::WritableStorage, public rosbag2_storage::ReadableStorage
{
public:
  ~TestPlugin() override;
  bool write(const char * data, size_t size) override;
  bool read_next(const char * buffer, size_t & size) override;

  void open_for_reading(const std::string & uri) override;
  void open_for_writing(const std::string & uri) override;
  rosbag2_storage::BagInfo info() override;
};

#endif  // TEST_PLUGIN_HPP_
