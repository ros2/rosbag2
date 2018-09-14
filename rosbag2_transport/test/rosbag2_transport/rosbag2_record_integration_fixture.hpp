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

#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_transport/rosbag2_transport.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_exception.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.hpp"

#include "rosbag2_test_fixture.hpp"
#include "test_memory_management.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

#ifndef ROSBAG2_TRANSPORT__ROSBAG2_RECORD_INTEGRATION_FIXTURE_HPP_
#define ROSBAG2_TRANSPORT__ROSBAG2_RECORD_INTEGRATION_FIXTURE_HPP_

class RosBag2RecordIntegrationTestFixture : public Rosbag2TestFixture
{
public:
  RosBag2RecordIntegrationTestFixture()
  : Rosbag2TestFixture()
  {
    rosbag2_storage_plugins::SqliteWrapper
    db(database_name_, rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE);

    rclcpp::init(0, nullptr);
    publisher_node_ = rclcpp::Node::make_shared("publisher_node");
  }

  void start_recording(std::vector<std::string> topics)
  {
    // the future object returned from std::async needs to be stored not to block the execution
    future_ = std::async(
      std::launch::async, [this, topics]() {
        rosbag2_transport::Rosbag2Transport rosbag2_transport;
        rosbag2_transport.record(database_name_, topics);
      });
  }

  void start_recording_all_topics()
  {
    // the future object returned from std::async needs to be stored not to block the execution
    future_ = std::async(
      std::launch::async, [this]() {
        rosbag2_transport::Rosbag2Transport rosbag2_transport;
        std::this_thread::sleep_for(2s);
        rosbag2_transport.record(database_name_);
      });
  }

  void stop_recording()
  {
    rclcpp::shutdown();
    future_.get();
  }

  void wait_for_publishers_to_stop()
  {
    for (auto & future : publisher_futures_) {
      future.get();
    }
  }

  template<typename MessageT>
  void start_publishing(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> message,
    const std::string & topic_name, size_t number_expected_messages)
  {
    publisher_futures_.push_back(std::async(
        std::launch::async, [this, message, topic_name, number_expected_messages]() {
          auto publisher = publisher_node_->create_publisher<MessageT>(topic_name);

          rosbag2_storage_plugins::SqliteWrapper
          db(database_name_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
          while (rclcpp::ok() && count_stored_messages(db, topic_name) < number_expected_messages) {
            publisher->publish(message->serialized_data.get());
            // rate limiting
            std::this_thread::sleep_for(50ms);
          }
        }
    ));
  }

  size_t
  count_stored_messages(rosbag2_storage_plugins::SqliteWrapper & db, const std::string & topic_name)
  {
    // protect against concurrent writes (from recording) that may make the count query throw.
    while (true) {
      try {
        return count_stored_messages_in_db(db, topic_name);
      } catch (const rosbag2_storage_plugins::SqliteException & e) {
        (void) e;
      }
      std::this_thread::sleep_for(50ms);
    }
  }

  size_t count_stored_messages_in_db(
    rosbag2_storage_plugins::SqliteWrapper & db, const std::string & topic_name)
  {
    auto row = db.prepare_statement(
      "SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name = 'topics';")
      ->execute_query<int>().get_single_line();
    if (std::get<0>(row) == 0) {
      return 0;
    }
    auto message_count = db.prepare_statement(
      "SELECT COUNT(*) "
      "FROM messages LEFT JOIN topics ON messages.topic_id = topics.id "
      "WHERE topics.name = ?;")->bind(topic_name)->execute_query<int>().get_single_line();
    return static_cast<size_t>(std::get<0>(message_count));
  }

  template<typename MessageT>
  std::shared_ptr<MessageT> deserialize_message(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
  {
    return memory_.deserialize_message<MessageT>(message->serialized_data);
  }

  template<typename MessageT>
  std::vector<std::shared_ptr<MessageT>> filter_messages(
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages,
    const std::string & topic)
  {
    std::vector<std::shared_ptr<MessageT>> filtered_messages;
    for (const auto & message : messages) {
      if (message->topic_name == topic) {
        filtered_messages.push_back(deserialize_message<MessageT>(message));
      }
    }
    return filtered_messages;
  }

  rclcpp::Node::SharedPtr publisher_node_;
  test_helpers::TestMemoryManagement memory_;
  std::vector<std::future<void>> publisher_futures_;
  std::future<void> future_;
};

#endif  // ROSBAG2_TRANSPORT__ROSBAG2_RECORD_INTEGRATION_FIXTURE_HPP_
