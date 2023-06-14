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
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/wait_for.hpp"

#include "rosbag2_transport/recorder.hpp"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "record_integration_fixture.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "mock_recorder.hpp"

class ClockPublisher
{
public:
  ClockPublisher(std::chrono::milliseconds pub_period, int64_t time_value)
  {
    node_ = std::make_shared<rclcpp::Node>(
      "clock_pub_node",
      rclcpp::NodeOptions().start_parameter_event_publisher(false).enable_rosout(false));
    pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(1));

    rclcpp::Time time{time_value};
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = time;

    timer_ = node_->create_wall_timer(
      pub_period,
      [&, clock_msg]() {
        pub_->publish(clock_msg);
      });
    exec_.add_node(node_);
    spin_thread_ = std::thread(
      [this]() {
        exec_.spin();
      });
  }

  ~ClockPublisher()
  {
    exec_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::thread spin_thread_;
};

TEST_F(RecordIntegrationTestFixture, record_all_with_sim_time)
{
  const int64_t time_value{123456789};
  const std::string clock_topic = "/clock";
  const std::string string_topic = "/string_topic";
  auto string_message = get_messages_strings()[0];
  string_message->string_value = "Hello World";

  // publishes /clock messages every 100ms until shutdown
  auto clock_pub = ClockPublisher(std::chrono::milliseconds(100), time_value);

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(string_topic, string_message, 5);

  rosbag2_transport::RecordOptions record_options =
  {
    false, false, {string_topic, clock_topic}, "rmw_format", 100ms
  };
  record_options.use_sim_time = true;
  auto recorder = std::make_shared<MockRecorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  ASSERT_TRUE(pub_manager.wait_for_matched(string_topic.c_str()));

  ASSERT_TRUE(recorder->wait_for_topic_to_be_discovered(string_topic));

  ASSERT_TRUE(recorder->topic_available_for_recording(string_topic));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = 10;
  auto ret = rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(5),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() >= expected_messages;
    });
  auto recorded_messages = mock_writer.get_messages();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time. " <<
    "recorded messages = " << recorded_messages.size();
  stop_spinning();

  auto messages_per_topic = mock_writer.messages_per_topic();
  EXPECT_EQ(messages_per_topic[string_topic], 5u);

  EXPECT_THAT(recorded_messages, SizeIs(Ge(expected_messages)));

  std::vector<rosbag2_storage::SerializedBagMessageSharedPtr> string_messages;
  for (const auto & message : recorded_messages) {
    if (message->topic_name == string_topic) {
      string_messages.push_back(message);
    }
  }
  // check that the timestamp is same as the clock message
  EXPECT_THAT(string_messages[0]->time_stamp, time_value);
}
