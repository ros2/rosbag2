// Copyright 2022, Foxglove Technologies Inc.
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

#ifndef ROSBAG2_TEST_COMMON__TESTED_STORAGE_IDS_HPP_
#define ROSBAG2_TEST_COMMON__TESTED_STORAGE_IDS_HPP_

#include <array>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace rosbag2_test_common
{
static const std::array<std::string, 2> kTestedStorageIDs = {
  "sqlite3",
  "mcap",
};

static const std::array<std::pair<std::string, std::string>, 2> kTestedStorageIDsWithExtensions = {
  {
    {"sqlite3", ".db3"},
    {"mcap", ".mcap"}
  }
};

static const std::unordered_map<std::string, std::string> kTestedStorageIDsToExtensions = {
  {"sqlite3", ".db3"},
  {"mcap", ".mcap"}
};

/// \brief Get the filename for a bag file based on the storage id.
/// The filename is constructed as <name><extension>, where the extension is determined by the
/// storage id.
inline std::string
bag_filename_for_storage_id(const std::string & name, const std::string & storage_id)
{
  for (const auto & [id, ext] : kTestedStorageIDsWithExtensions) {
    if (id == storage_id) {
      return name + ext;
    }
  }
  throw std::runtime_error("unknown storage id: " + storage_id);
}

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__TESTED_STORAGE_IDS_HPP_
