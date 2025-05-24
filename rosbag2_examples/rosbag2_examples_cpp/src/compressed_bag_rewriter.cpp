// Copyright 2025 Open Source Robotics Foundation
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

#include <vector>

#include "rosbag2_transport/bag_rewrite.hpp"

int main(int, char **)
{
  rosbag2_storage::StorageOptions input_storage;
  input_storage.uri = "input_bag";

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = "output_bag";

  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = true;
  record_options.compression_format = "zstd";
  record_options.compression_mode = "message";

  std::vector<rosbag2_storage::StorageOptions> input_bags;
  input_bags.push_back(input_storage);

  std::vector<std::pair<rosbag2_storage::StorageOptions,
    rosbag2_transport::RecordOptions>> output_bags;
  output_bags.push_back({output_storage, record_options});

  rosbag2_transport::bag_rewrite(input_bags, output_bags);

  return 0;
}
