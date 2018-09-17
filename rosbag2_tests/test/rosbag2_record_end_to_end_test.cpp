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
#include <iostream>
#include <memory>
#include <future>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"  // rclcpp must be included before the Windows specific includes.

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#endif

#include "test_msgs/msg/primitives.hpp"
#include "test_msgs/msg/static_array_primitives.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

#ifdef _WIN32
using ProcessHandle = HANDLE;
#else
using ProcessHandle = int;
#endif

class EndToEndTestFixture : public Test
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
#ifdef _WIN32
    DeleteFileA(database_name_.c_str());
#else
    // TODO(botteroa-si): once filesystem::remove_all() can be used, this line can be removed and
    // the ful directory can be deleted in remove_temporary_dir()
    remove(database_name_.c_str());
#endif
    rclcpp::shutdown();
  }

  static void SetUpTestCase()
  {
    char template_char[] = "tmp_test_dir.XXXXXX";
#ifdef _WIN32
    char temp_path[255];
    GetTempPathA(255, temp_path);
    _mktemp_s(template_char, strnlen(template_char, 20) + 1);
    temporary_dir_path_ = std::string(temp_path) + std::string(template_char);
    _mkdir(temporary_dir_path_.c_str());
#else
    char * dir_name = mkdtemp(template_char);
    temporary_dir_path_ = dir_name;
#endif
  }

  static void TearDownTestCase()
  {
    remove_temporary_dir();
  }

  static void remove_temporary_dir()
  {
#ifdef _WIN32
    // TODO(botteroa-si): find a way to delete a not empty directory in Windows, so that we don't
    // need the Windows line in the fixture destructor anymore.
    RemoveDirectoryA(temporary_dir_path_.c_str());
#else
    remove(temporary_dir_path_.c_str());
#endif
  }

  ProcessHandle record_all_topics()
  {
#ifdef _WIN32
    auto h_job = CreateJobObject(nullptr, nullptr);
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION info{};
    info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    SetInformationJobObject(h_job, JobObjectExtendedLimitInformation, &info, sizeof(info));

    STARTUPINFO start_up_info{};
    PROCESS_INFORMATION process_info{};
    CreateProcess(nullptr, "ros2 bag record -a", nullptr, nullptr, false, 0, nullptr,
      temporary_dir_path_.c_str(),
      &start_up_info, &process_info);

    AssignProcessToJobObject(h_job, process_info.hProcess);
    CloseHandle(process_info.hProcess);
    CloseHandle(process_info.hThread);
    std::this_thread::sleep_for(3s);  // we must wait for rosbag2 to start.
    return h_job;
#else
    auto process_id = fork();
    if (process_id == 0) {
      setpgid(getpid(), getpid());
      chdir(temporary_dir_path_.c_str());
      system("ros2 bag record -a");
    } else {
      std::this_thread::sleep_for(3s);  // we must wait for rosbag2 to start.
    }
    return process_id;
#endif
  }

  void play_bag()
  {
#ifdef _WIN32
    STARTUPINFO start_up_info{};
    PROCESS_INFORMATION process_info{};
    CreateProcess(nullptr, "ros2 bag play test.bag", nullptr, nullptr, false, 0, nullptr,
      temporary_dir_path_.c_str(),
      &start_up_info, &process_info);
#else
    chdir(temporary_dir_path_.c_str());
    system("ros2 bag play test.bag");
#endif
  }

  std::future<void> publish_test_message()
  {
    return async(
      std::launch::async, [this]() {
        std::string topic_name = "/test_topic";
        auto publisher_node = std::make_shared<rclcpp::Node>("publisher_node");
        auto publisher =
        publisher_node->create_publisher<test_msgs::msg::Primitives>(topic_name);
        auto message = get_messages_primitives()[0];
        message->string_value = "test";

        rosbag2_storage_plugins::SqliteWrapper
        db(database_name_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
        while (rclcpp::ok() && count_stored_messages(db, topic_name) < 3) {
          publisher->publish(message);
          // rate limiting
          std::this_thread::sleep_for(50ms);
        }
      });
  }

  std::future<std::vector<std::string>> subscribe_and_wait_for_one_test_message()
  {
    return async(
      std::launch::async, []() -> std::vector<std::string> {
        std::vector<std::string> messages;
        int counter = 0;
        auto subscriber_node = std::make_shared<rclcpp::Node>("subscriber_node");
        auto subscription = subscriber_node->create_subscription<test_msgs::msg::Primitives>(
          "test_topic",
          [&messages, &counter](test_msgs::msg::Primitives::SharedPtr msg) {
            messages.push_back(msg->string_value);
            counter++;
          });
        while (counter < 1) {
          rclcpp::spin_some(subscriber_node);
        }
        return messages;
      });
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

  void stop_recording(ProcessHandle process_handle)
  {
#ifdef _WIN32
    CloseHandle(process_handle);
#else
    kill(-process_handle, SIGTERM);
#endif
  }

  std::string database_name_;
  static std::string temporary_dir_path_;
};

std::string EndToEndTestFixture::temporary_dir_path_ = "";  // NOLINT

TEST_F(EndToEndTestFixture, record_end_to_end_test) {
  auto id = record_all_topics();
  publish_test_message().get();

  stop_recording(id);

  auto recorded_messages = get_messages();
  ASSERT_THAT(recorded_messages, SizeIs(Ge(3u)));
  EXPECT_THAT(recorded_messages[0]->topic_name, Eq("/test_topic"));
}
