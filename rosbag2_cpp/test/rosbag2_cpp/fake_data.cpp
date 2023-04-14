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
#include <utility>
#include <vector>

#include "fake_data.hpp"

#include "rosbag2_storage/message_definition.hpp"
#include "rosbag2_storage/ros_helper.hpp"

void write_sample_split_bag(
  const rosbag2_storage::StorageOptions & storage_options,
  const std::vector<std::pair<rcutils_time_point_value_t, uint32_t>> & fake_messages,
  size_t split_every)
{
  std::string topic_name = "testtopic";

  rosbag2_cpp::writers::SequentialWriter writer{};
  writer.open(storage_options, rosbag2_cpp::ConverterOptions{});

  writer.create_topic(
  {
    topic_name,
    "test_msgs/msg/ByteMultiArray",
    "cdr",
    "",
    ""
  },
  {
    "test_msgs/msg/ByteMultiArray",
    "ros2msg",
    "# This was originally provided as an example message.\n"
    "# It is deprecated as of Foxy\n"
    "# It is recommended to create your own semantically meaningful message.\n"
    "\n"
    "# Please look at the MultiArrayLayout message definition for\n"
    "# documentation on all multiarrays.\n"
    "\n"
    "MultiArrayLayout  layout        # specification of data layout\n"
    "byte[]            data          # array of data"
  });
  for (size_t i = 0; i < fake_messages.size(); i++) {
    if (i > 0 && (i % split_every == 0)) {
      writer.split_bagfile();
    }

    const auto message = fake_messages[i];
    rcutils_time_point_value_t time_stamp = message.first;
    uint32_t value = message.second;

    auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    msg->serialized_data = rosbag2_storage::make_serialized_message(&value, sizeof(value));
    msg->time_stamp = time_stamp;
    msg->topic_name = topic_name;
    writer.write(msg);
  }
  writer.close();
}
