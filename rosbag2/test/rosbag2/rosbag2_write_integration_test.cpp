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

#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/rosbag2.hpp"
#include "std_msgs/msg/string.hpp"

#include "rosbag2_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

// TODO(Martin-Idel-SI): merge with rosbag2_read_integration_test once signal handling is sorted out
class RosBag2IntegrationTestFixture : public Rosbag2TestFixture
{
public:
  RosBag2IntegrationTestFixture()
  : Rosbag2TestFixture(), counter_(0)
  {}

  void record_message(const std::string & db_name)
  {
    rosbag2::Rosbag2 rosbag2;
    rosbag2.record(db_name, "string_topic", [this]() {
        counter_++;
      });
  }

  void start_recording()
  {
    rclcpp::init(0, nullptr);
    // the future object returned from std::async needs to be stored not to block the execution
    future_ = std::async(
      std::launch::async, [this]() {
        record_message(database_name_);
      });
  }

  void stop_recording()
  {
    rclcpp::shutdown();
  }

  void publish_messages(std::vector<std::string> messages)
  {
    size_t expected_counter_value = messages.size();
    auto node = std::make_shared<rclcpp::Node>("publisher_node");
    auto publisher = node->create_publisher<std_msgs::msg::String>("string_topic");
    auto timer = node->create_wall_timer(50ms, [this, publisher, messages]() {
          auto msg = std_msgs::msg::String();
          msg.data = messages[counter_];
          publisher->publish(msg);
        });

    while (counter_ < expected_counter_value) {
      rclcpp::spin_some(node);
    }
  }

  std::atomic<size_t> counter_;
  std::future<void> future_;
};

TEST_F(RosBag2IntegrationTestFixture, published_messages_are_recorded)
{
  std::string message_to_publish = "test_message";

  start_recording();
  publish_messages({message_to_publish});
  stop_recording();

  auto recorded_messages = get_messages(database_name_);

  ASSERT_THAT(recorded_messages, Not(IsEmpty()));
}
