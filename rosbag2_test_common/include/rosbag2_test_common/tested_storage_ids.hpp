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
#include <string>


namespace rosbag2_test_common
{
static const std::array<std::string, 2> kTestedStorageIDs = {
  "sqlite3",
  "mcap",
};

std::string bag_filename_for_storage_id(const std::string & name, const std::string & storage_id)
{
  const std::array<std::string, 2> extensions = {".db3", ".mcap"};
  static_assert(kTestedStorageIDs.size() == extensions.size());
  for (size_t i = 0; i < extensions.size(); ++i) {
    if (kTestedStorageIDs[i] == storage_id) {
      return name + extensions[i];
    }
  }
  throw std::runtime_error("unknown storage id: " + storage_id);
}

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__TESTED_STORAGE_IDS_HPP_
