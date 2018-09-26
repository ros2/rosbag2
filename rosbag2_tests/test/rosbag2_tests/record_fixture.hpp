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

#ifndef ROSBAG2_TESTS__RECORD_FIXTURE_HPP_
#define ROSBAG2_TESTS__RECORD_FIXTURE_HPP_

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
#include "rosbag2_storage/filesystem_helpers.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/publisher_manager.hpp"
#include "rosbag2_test_common/memory_management.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

#ifdef _WIN32
struct Process
{
  PROCESS_INFORMATION process_info;
  HANDLE job_handle;
};

using ProcessHandle = Process;
#else
using ProcessHandle = int;
#endif

class RecordFixture : public TemporaryDirectoryFixture
{
public:
  RecordFixture()
  {
    bag_path_ = temporary_dir_path_ + rosbag2_storage::separator() + "bag";
    database_path_ = bag_path_ + rosbag2_storage::separator() + "bag.db3";
    std::cout << "Database " << database_path_ << " in " << temporary_dir_path_ << std::endl;
    rclcpp::init(0, nullptr);
  }

  ~RecordFixture() override
  {
    rclcpp::shutdown();
  }

  ProcessHandle start_execution(const std::string & command)
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
      nullptr,
      &start_up_info,
      &process_info);

    AssignProcessToJobObject(h_job, process_info.hProcess);
    Process process;
    process.process_info = process_info;
    process.job_handle = h_job;

    delete[] command_char;
    return process;
#else
    auto process_id = fork();
    if (process_id == 0) {
      setpgid(getpid(), getpid());
      int return_code = system(command.c_str());
      EXPECT_THAT(return_code, Eq(0));  // this call will make sure that the process does execute
      // without issues before it is killed by the user in the test or, in case it runs until
      // completion, that it has correctly executed.
    }
    return process_id;
#endif
  }

  void wait_for_db()
  {
    while (true) {
      try {
        std::this_thread::sleep_for(50ms);  // wait a bit to not query constantly
        rosbag2_storage_plugins::SqliteWrapper
          db(database_path_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
        return;
      } catch (const rosbag2_storage_plugins::SqliteException & ex) {
        (void) ex;  // still waiting
      }
    }
  }

  void stop_execution(ProcessHandle handle)
  {
#ifdef _WIN32
    DWORD exit_code;
    GetExitCodeProcess(handle.process_info.hProcess, &exit_code);
    // 259 indicates that the process is still active: we want to make sure that the process is
    // still running properly before killing it.
    EXPECT_THAT(exit_code, Eq(259));

    CloseHandle(handle.process_info.hProcess);
    CloseHandle(handle.process_info.hThread);
    CloseHandle(handle.job_handle);
#else
    kill(-handle, SIGTERM);
#endif
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
  std::vector<std::shared_ptr<MessageT>> get_messages_for_topic(const std::string & topic)
  {
    auto all_messages = get_messages();
    auto topic_messages = std::vector<std::shared_ptr<MessageT>>();
    for (const auto & msg : all_messages) {
      if (msg->topic_name == topic) {
        topic_messages.push_back(
          memory_management_.deserialize_message<MessageT>(msg->serialized_data));
      }
    }
    return topic_messages;
  }

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> get_messages()
  {
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> table_msgs;
    auto storage = std::make_shared<rosbag2_storage_plugins::SqliteStorage>();
    storage->open(bag_path_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);

    while (storage->has_next()) {
      table_msgs.push_back(storage->read_next());
    }

    return table_msgs;
  }

  std::string bag_path_;
  std::string database_path_;
  PublisherManager pub_man_;
  MemoryManagement memory_management_;
};


#endif  // ROSBAG2_TESTS__RECORD_FIXTURE_HPP_
