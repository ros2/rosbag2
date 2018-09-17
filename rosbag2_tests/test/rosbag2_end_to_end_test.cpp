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

using namespace ::testing;  // NOLINT

class EndToEndTestFixture : public Test
{
public:
  EndToEndTestFixture()
  : database_name_(std::string(UnitTest::GetInstance()->current_test_info()->name()) + ".db3")
  {
    std::string system_separator = "/";
#ifdef _WIN32
    system_separator = "\\";
#endif
    database_name_ = temporary_dir_path_ + system_separator + database_name_;
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

  void record_all_topics()
  {
#ifdef _WIN32
    auto h_job = CreateJobObject(nullptr, nullptr);
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION info{};
    info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    SetInformationJobObject(h_job, JobObjectExtendedLimitInformation, &info, sizeof(info));

    STARTUPINFO start_up_info{};
    PROCESS_INFORMATION process_info{};
    CreateProcess(nullptr, "ros2 bag record -a", nullptr, nullptr, false, 0, nullptr, nullptr,
      &start_up_info, &process_info);

    AssignProcessToJobObject(h_job, process_info.hProcess);
    CloseHandle(process_info.hProcess);
    CloseHandle(process_info.hThread);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    CloseHandle(h_job);
#else
    auto process_id = fork();
    if (process_id == 0) {
      setpgid(getpid(), getpid());
      system(("cd " + temporary_dir_path_ + " && ros2 bag record -a").c_str());
    } else {
      // Here we wait to allow rosbag2 to record some messages before killing the process.
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      kill(-process_id, SIGINT);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      kill(-process_id, SIGTERM);
    }
#endif
  }

  void play_bag()
  {
#ifdef _WIN32
    STARTUPINFO start_up_info{};
    PROCESS_INFORMATION process_info{};
    CreateProcess(nullptr, "ros2 bag play test.bag", nullptr, nullptr, false, 0, nullptr, nullptr,
      &start_up_info, &process_info);
#else
    system(("cd " + temporary_dir_path_ + " && ros2 bag play test.bag").c_str());
#endif
  }

  std::future<void> publish_test_message()
  {
    return async(std::launch::async, []() {
               auto publisher_node = std::make_shared<rclcpp::Node>("publisher_node");
               auto publisher =
               publisher_node->create_publisher<test_msgs::msg::Primitives>("test_topic");
               auto message = get_messages_primitives()[0];
               message->string_value = "test";
               for (int i = 0; i < 10; i++) {
                 publisher->publish(message);
                 std::this_thread::sleep_for(std::chrono::milliseconds(200));
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

  std::string database_name_;
  static std::string temporary_dir_path_;
};

std::string EndToEndTestFixture::temporary_dir_path_ = "";  // NOLINT

TEST_F(EndToEndTestFixture, messages_are_recorder_and_replayed_correctly) {
  auto publisher_future = publish_test_message();
  record_all_topics();
  publisher_future.wait();

  auto subscriber_future = subscribe_and_wait_for_one_test_message();
  play_bag();
  auto messages = subscriber_future.get();

  ASSERT_THAT(messages, Not(IsEmpty()));
  EXPECT_THAT(messages[0], Eq("test"));
}
