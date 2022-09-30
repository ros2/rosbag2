// Copyright 2022, Foxglove Technologies. All rights reserved.
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

#ifndef ROSBAG2_CPP__FAKE_DATA_HPP_
#define ROSBAG2_CPP__FAKE_DATA_HPP_

#include <vector>

#include "rosbag2_cpp/writers/sequential_writer.hpp"

class ManualSplitSequentialWriter : public rosbag2_cpp::writers::SequentialWriter
{
public:
  using rosbag2_cpp::writers::SequentialWriter::split_bagfile;
};

void write_sample_split_bag(
  const rosbag2_storage::StorageOptions & storage_options,
  const std::vector<std::vector<rcutils_time_point_value_t>> & message_timestamps_by_file);

#endif  // ROSBAG2_CPP__FAKE_DATA_HPP_
