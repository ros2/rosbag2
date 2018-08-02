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

#include "rosbag2_storage/storage_interface.hpp"

class TestPlugin : public rosbag2_storage::StorageInterface
{
public:
  TestPlugin()
  {
    fprintf(stderr, "testplugin instantiated\n");
  }

  ~TestPlugin()
  {
    fprintf(stderr, "testplugin destroyed\n");
  }

  rosbag2_storage::StorageHandle
  open(const std::string & file_path);

  bool
  close(rosbag2_storage::StorageHandle & storage_handle);

  void
  set_storage_identifier(const std::string & storage_identifier);

  std::string
  get_storage_identifier();

private:
  std::string storage_identifier_;
};

#endif  // TEST_PLUGIN_HPP_
