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

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rosbag2_transport/rosbag2_transport.hpp"
#include "rosbag2_transport/convert_options.hpp"
#include "rosbag2_transport/storage_options.hpp"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_transport_test_fixture.hpp"

TEST_F(Rosbag2TransportTestFixture, convert_writes_all_input_topics_and_messages)
{
  rosbag2_transport::StorageOptions out_options;
  out_options.uri = "uri_out";
  out_options.storage_id = "storage_id_out";

  rosbag2_transport::ConvertOptions convert_options;
  convert_options.rmw_serialization_format = "rmw_format_out";
  convert_options.compression_format = "";
  convert_options.compression_mode = "";

  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;
  auto primitive_message2 = get_messages_basic_types()[1];
  primitive_message2->bool_value = true;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", ""}
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 500, primitive_message1),
    serialize_test_message("topic1", 1000, primitive_message2)};

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  rosbag2_transport::Rosbag2Transport rosbag2_transport(reader_, writer_, info_);
  rosbag2_transport.convert(storage_options_, out_options, convert_options);

  MockSequentialWriter & writer =
    static_cast<MockSequentialWriter &>(writer_->get_implementation_handle());
  auto written_messages = writer.get_messages();
  auto written_topics = writer.get_topics();

  ASSERT_THAT(written_topics, SizeIs(1));
  EXPECT_THAT(written_topics.at("topic1").name, Eq("topic1"));
  EXPECT_THAT(written_topics.at("topic1").type, Eq("test_msgs/BasicTypes"));
  EXPECT_THAT(written_topics.at("topic1").serialization_format, Eq("rmw_format_out"));

  ASSERT_THAT(written_messages, SizeIs(2));
  EXPECT_THAT(written_messages.at(0)->topic_name, Eq("topic1"));
  EXPECT_THAT(written_messages.at(0)->time_stamp, Eq(500 * 1000 * 1000));
  auto basic_types_msg1 = memory_management_.deserialize_message<test_msgs::msg::BasicTypes>(
    written_messages.at(0)->serialized_data);
  EXPECT_THAT(
    *basic_types_msg1,
    Field(&test_msgs::msg::BasicTypes::int32_value, 42));

  EXPECT_THAT(written_messages.at(1)->topic_name, Eq("topic1"));
  EXPECT_THAT(written_messages.at(1)->time_stamp, Eq(1000 * 1000 * 1000));
  auto basic_types_msg2 = memory_management_.deserialize_message<test_msgs::msg::BasicTypes>(
    written_messages.at(1)->serialized_data);
  EXPECT_THAT(
    *basic_types_msg2,
    Field(&test_msgs::msg::BasicTypes::bool_value, true));
}
