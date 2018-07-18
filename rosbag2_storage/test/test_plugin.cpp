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

#include <string>

#include "pluginlib/class_list_macros.hpp"

#include "test_plugin.hpp"

rosbag2_storage::StorageHandle
TestPlugin::open(const std::string & file_path)
{
  fprintf(stderr, "opening file path %s\n", file_path.c_str());
  rosbag2_storage::StorageHandle handle;
  handle.file_path = file_path;

  return handle;
}

bool
TestPlugin::close(rosbag2_storage::StorageHandle & storage_handle)
{
  fprintf(stderr, "closing file handle %s\n", storage_handle.file_path.c_str());
  return true;
}

void
TestPlugin::set_storage_identifier(const std::string & storage_identifier)
{
  storage_identifier_ = storage_identifier;
}

std::string
TestPlugin::get_storage_identifier()
{
  return storage_identifier_;
}

PLUGINLIB_EXPORT_CLASS(TestPlugin, rosbag2_storage::StorageInterface)
