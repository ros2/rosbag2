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

#include <memory>
#include <string>
#include <vector>

#include "fake_data.hpp"

#include "rosbag2_storage/ros_helper.hpp"

void write_sample_split_bag(
  const rosbag2_storage::StorageOptions & storage_options,
  const std::vector<std::vector<rcutils_time_point_value_t>> & message_timestamps_by_file)
{
  std::string msg_content = "Hello";
  auto msg_length = msg_content.length();
  std::shared_ptr<rcutils_uint8_array_t> fake_data = rosbag2_storage::make_serialized_message(
    msg_content.c_str(), msg_length);
  std::string topic_name = "testtopic";

  ManualSplitSequentialWriter writer{};
  writer.open(storage_options, rosbag2_cpp::ConverterOptions{});
  writer.create_topic(
  {
    topic_name,
    "test_msgs/ByteMultiArray",
    "cdr",
    ""
  });
  for (const auto & file_messages : message_timestamps_by_file) {
    for (const auto time_stamp : file_messages) {
      auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      msg->serialized_data = fake_data;
      msg->time_stamp = time_stamp;
      msg->topic_name = topic_name;
      writer.write(msg);
    }
    writer.split_bagfile();
  }
  writer.close();
}
