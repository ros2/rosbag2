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
#include <future>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"  // rclcpp must be included before the Windows specific includes.

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#else
# include <unistd.h>
#endif

#include "test_msgs/msg/primitives.hpp"
#include "test_msgs/msg/static_array_primitives.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"
#include "temporary_directory_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

#ifdef _WIN32
using ProcessHandle = HANDLE;
#else
using ProcessHandle = int;
#endif

class EndToEndTestFixture : public TemporaryDirectoryFixture
{
public:
  EndToEndTestFixture()
  {
    std::string system_separator = "/";
#ifdef _WIN32
    system_separator = "\\";
#endif
    database_name_ = temporary_dir_path_ + system_separator + "test.bag";
    std::cout << "Database name: " << database_name_ << std::endl;
    rclcpp::init(0, nullptr);
  }

  ~EndToEndTestFixture() override
  {
    rclcpp::shutdown();
  }

  void start_recording(const std::string & command)
  {
#ifdef _WIN32
    auto h_job = CreateJobObject(nullptr, nullptr);
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION info{};
    info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    SetInformationJobObject(h_job, JobObjectExtendedLimitInformation, &info, sizeof(info));

    STARTUPINFO start_up_info{};
    PROCESS_INFORMATION process_info{};

    size_t length = strlen(command.c_str());
    TCHAR * command_char = new TCHAR[length + 1];
    memcpy(command_char, command.c_str(), length + 1);

    CreateProcess(
      nullptr,
      command_char,
      nullptr,
      nullptr,
      false,
      0,
      nullptr,
      temporary_dir_path_.c_str(),
      &start_up_info,
      &process_info);

    AssignProcessToJobObject(h_job, process_info.hProcess);
    CloseHandle(process_info.hProcess);
    CloseHandle(process_info.hThread);
    delete[] command_char;
    wait_for_db();
    bag_handle_ = h_job;
#else
    auto process_id = fork();
    if (process_id == 0) {
      setpgid(getpid(), getpid());
      chdir(temporary_dir_path_.c_str());
      system(command.c_str());
    } else {
      wait_for_db();
      bag_handle_ = process_id;
    }
#endif
  }

  void wait_for_db()
  {
    while (true) {
      try {
        std::this_thread::sleep_for(50ms);  // wait a bit to not query constantly
        rosbag2_storage_plugins::SqliteWrapper
          db(database_name_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
        return;
      } catch (const rosbag2_storage_plugins::SqliteException & ex) {
        (void) ex;  // still waiting
      }
    }
  }

  void stop_recording()
  {
#ifdef _WIN32
    CloseHandle(bag_handle_);
#else
    kill(-bag_handle_, SIGTERM);
#endif
  }

  template<class T>
  auto create_publisher(
    const std::string & topic_name, std::shared_ptr<T> message, size_t expected_messages = 0)
  {
    static int counter = 0;
    auto publisher_node = std::make_shared<rclcpp::Node>("publisher" + std::to_string(counter++));
    auto publisher = publisher_node->create_publisher<T>(topic_name);

    // We need to publish one message to set up the topic for discovery
    publisher->publish(message);

    return [this, publisher, topic_name, message, expected_messages]() {
             rosbag2_storage_plugins::SqliteWrapper
               db(database_name_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
             if (expected_messages != 0) {
               while (rclcpp::ok() && count_stored_messages(db, topic_name) < expected_messages) {
                 publisher->publish(message);
                 // rate limiting
                 std::this_thread::sleep_for(50ms);
               }
             } else {
               // Just publish a few messages - they should never be stored
               publisher->publish(message);
               std::this_thread::sleep_for(50ms);
               publisher->publish(message);
               std::this_thread::sleep_for(50ms);
               publisher->publish(message);
             }
           };
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

  void run_publishers(std::initializer_list<std::function<void()>> publishers)
  {
    std::vector<std::future<void>> futures;
    for (const auto & publisher : publishers) {
      futures.push_back(std::async(std::launch::async, publisher));
    }
    for (auto & publisher_future : futures) {
      publisher_future.get();
    }
  }

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> get_messages()
  {
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> table_msgs;
    auto storage = std::make_shared<rosbag2_storage_plugins::SqliteStorage>();
    storage->open(database_name_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);

    while (storage->has_next()) {
      table_msgs.push_back(storage->read_next());
    }

    return table_msgs;
  }

  template<typename T>
  inline
  const rosidl_message_type_support_t * get_message_typesupport(std::shared_ptr<T>)
  {
    return rosidl_typesupport_cpp::get_message_type_support_handle<T>();
  }

  template<typename T>
  inline
  std::shared_ptr<T> deserialize_message(std::shared_ptr<rmw_serialized_message_t> serialized_msg)
  {
    auto message = std::make_shared<T>();
    auto error = rmw_deserialize(
      serialized_msg.get(),
      get_message_typesupport(message),
      message.get());
    if (error != RCL_RET_OK) {
      throw std::runtime_error("Failed to deserialize");
    }
    return message;
  }

  ProcessHandle bag_handle_;
  std::string database_name_;
};

TEST_F(EndToEndTestFixture, record_end_to_end_test) {
  auto message = get_messages_primitives()[0];
  message->string_value = "test";
  size_t expected_test_messages = 3;
  auto test_publisher = create_publisher("/test_topic", message, expected_test_messages);

  auto wrong_message = get_messages_primitives()[0];
  wrong_message->string_value = "wrong_content";
  auto wrong_publisher = create_publisher("/wrong_topic", wrong_message);

  start_recording("ros2 bag record /test_topic");

  run_publishers({test_publisher, wrong_publisher});

  stop_recording();

  auto recorded_messages = get_messages();
  ASSERT_THAT(recorded_messages, SizeIs(Ge(expected_test_messages)));
  for (const auto & recorded_message : recorded_messages) {
    EXPECT_THAT(recorded_message->topic_name, Eq("/test_topic"));
    auto deserialized_message = deserialize_message<test_msgs::msg::Primitives>(
      recorded_message->serialized_data);
    EXPECT_THAT(deserialized_message->string_value, Eq("test"));
  }
}
