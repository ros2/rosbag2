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

// TODO(Martin-Idel-SI): merge w. rosbag2_write_integration_test once signal handling is sorted out
class RosBag2IntegrationTestFixture : public Rosbag2TestFixture
{
public:
  RosBag2IntegrationTestFixture()
  : Rosbag2TestFixture(), counter_(0)
  {}

  std::vector<std::string> subscribe_messages(size_t expected_messages_number)
  {
    std::vector<std::string> messages;
    auto node = std::make_shared<rclcpp::Node>("subscriber_node");
    auto subscription = node->create_subscription<std_msgs::msg::String>("string_topic",
        [this, &messages](const std_msgs::msg::String::ConstSharedPtr message) {
          messages.emplace_back(message->data);
          counter_++;
        }, 10);

    while (counter_ < expected_messages_number) {
      rclcpp::spin_some(node);
    }
    return messages;
  }

  void launch_subscriber(size_t expected_messages_number)
  {
    subscriber_future_ = std::async(std::launch::async, [this, expected_messages_number] {
          return subscribe_messages(expected_messages_number);
        });
  }

  void play_bag(std::string database_name, std::string topic)
  {
    rosbag2::Rosbag2 rosbag2;
    rosbag2.play(database_name, topic);
  }

  std::atomic<size_t> counter_;
  std::future<std::vector<std::string>> subscriber_future_;
};

TEST_F(RosBag2IntegrationTestFixture, recorded_messages_are_played)
{
  rclcpp::init(0, nullptr);

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
  for (int i : {1, 2, 3}) {
    (void) i;
    auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    auto payload = new rcutils_char_array_t;
    *payload = rcutils_get_zero_initialized_char_array();
    payload->allocator = rcutils_get_default_allocator();
    // TODO(Martin-Idel-SI) The real serialized string message has 8 leading chars in CDR
    std::string string_message_to_publish = "bbbbbbbbHello World";
    auto ret = rcutils_char_array_resize(payload, strlen(string_message_to_publish.c_str()) + 1);
    if (ret != RCUTILS_RET_OK) {
      FAIL() << " Failed to resize serialized bag message";
    }
    memcpy(payload->buffer,
      string_message_to_publish.c_str(),
      strlen(string_message_to_publish.c_str()) + 1);

    msg->serialized_data = std::shared_ptr<rcutils_char_array_t>(payload,
        [](rcutils_char_array_t * msg) {
          auto error = rcutils_char_array_fini(msg);
          delete msg;
          if (error != RCUTILS_RET_OK) {
            FAIL() << " Failed to destroy serialized bag message";
          }
        });
    messages.push_back(msg);
  }
  ASSERT_NO_THROW(write_messages(database_name_, messages));

  // Due to a problem related to the subscriber, we play many (3) messages but make the subscriber
  // node spin only until 2 have arrived. Hence the 2 as `launch_subscriber()` argument.
  launch_subscriber(2);
  play_bag(database_name_, "string_topic");

  auto replayed_messages = subscriber_future_.get();
  ASSERT_THAT(replayed_messages, SizeIs(2));
  ASSERT_THAT(replayed_messages[0], Eq("Hello World"));
  ASSERT_THAT(replayed_messages[1], Eq("Hello World"));

  rclcpp::shutdown();
}
