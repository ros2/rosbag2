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

#ifndef TEST_READ_ONLY_PLUGIN_HPP_
#define TEST_READ_ONLY_PLUGIN_HPP_

#include <string>

#include "rosbag2_storage/readable_storage.hpp"

class TestReadOnlyPlugin : public rosbag2_storage::ReadableStorage
{
public:
  ~TestReadOnlyPlugin() override;
  void read_next(std::string & message) override;
  void open_readonly(const std::string & uri) override;
  rosbag2_storage::BagInfo info() override;
};

#endif  // TEST_READ_ONLY_PLUGIN_HPP_
