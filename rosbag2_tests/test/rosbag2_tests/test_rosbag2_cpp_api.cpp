// Copyright 2020, Bosch Software Innovations GmbH.
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

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

#include "test_msgs/msg/basic_types.hpp"

TEST(TestRosbag2CPPAPI, minimal_writer_example)
{
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;
  test_msg.float64_value = 12345.6789;
  rclcpp::SerializedMessage serialized_msg;

  rclcpp::Serialization<TestMsgT> serialization;
  serialization.serialize_message(&test_msg, &serialized_msg);

  auto rosbag_directory = rcpputils::fs::path("test_rosbag2_writer_api_bag");
  // in case the bag was previously not cleaned up
  rcpputils::fs::remove_all(rosbag_directory);

  {
    rosbag2_cpp::Writer writer;
    writer.open(rosbag_directory.string());

    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    auto ret = rcutils_system_time_now(&bag_message->time_stamp);
    if (ret != RCL_RET_OK) {
      FAIL() << "couldn't assign time rosbag message";
    }

    rosbag2_storage::TopicMetadata tm;
    tm.name = "/my/test/topic";
    tm.type = "test_msgs/msg/BasicTypes";
    tm.serialization_format = "cdr";
    writer.create_topic(tm);

    bag_message->topic_name = tm.name;
    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      &serialized_msg.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});

    writer.write(bag_message);
    // close on scope exit
  }

  {
    rosbag2_cpp::Reader reader;
    reader.open(rosbag_directory.string());
    while (reader.has_next()) {
      auto bag_message = reader.read_next();

      TestMsgT extracted_test_msg;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization.deserialize_message(
        &extracted_serialized_msg, &extracted_test_msg);

      EXPECT_EQ(test_msg, extracted_test_msg);
    }
    // close on scope exit
  }

  // remove the rosbag again after the test
  EXPECT_TRUE(rcpputils::fs::remove_all(rosbag_directory));
}
