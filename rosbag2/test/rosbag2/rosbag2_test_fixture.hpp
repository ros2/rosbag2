/*
 *  Copyright (c) 2018,  Bosch Software Innovations GmbH.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef ROSBAG2__ROSBAG2_TEST_FIXTURE_HPP_
#define ROSBAG2__ROSBAG2_TEST_FIXTURE_HPP_

#include <gtest/gtest.h>
#include <sqlite3.h>

#include <cstdio>
#if defined(_WIN32)
#include <stdlib.h>
#endif

#include <string>
#include <vector>

using namespace ::testing;  // NOLINT

class Rosbag2TestFixture : public Test
{
public:
  Rosbag2TestFixture()
  : database_name_(UnitTest::GetInstance()->current_test_info()->name())
  {
#if defined(__linux__) || defined(__APPLE__)
    database_name_ = temporary_dir_path_ + "/" + database_name_;
#elif defined(_WIN32)
    database_name_ = temporary_dir_path_ + "\\" + database_name_;
#endif
  }

  ~Rosbag2TestFixture() override
  {
#if defined(_WIN32)
    // TODO(botteroa-si): remove once a nice way to delete a not empty directory is found.
    DeleteFileA(database_name_.c_str());
#endif
  }

  static void SetUpTestCase()
  {
#if defined(__linux__) || defined(__APPLE__)
    char template_char[] = "/tmp/tmp_test_dir.XXXXXX";
    char * dir_name = mkdtemp(template_char);
    temporary_dir_path_ = dir_name;
#elif defined(_WIN32)
    char dir_name[6];
    tmpnam_s(dir_name, 6);
    temporary_dir_path = std::string(dir_name);
#endif
  }

  static void TearDownTestCase()
  {
    remove_temporary_dir();
  }

  std::vector<std::string> get_messages(std::string db_name)
  {
    sqlite3 * database;
    sqlite3_open(db_name.c_str(), &database);

    std::vector<std::string> table_msgs;
    sqlite3_stmt * statement;
    std::string query = "SELECT * FROM messages";
    sqlite3_prepare_v2(database, query.c_str(), -1, &statement, nullptr);
    int result = sqlite3_step(statement);
    while (result == SQLITE_ROW) {
      table_msgs.emplace_back(reinterpret_cast<const char *>(sqlite3_column_text(statement, 1)));
      result = sqlite3_step(statement);
    }
    sqlite3_finalize(statement);
    sqlite3_close(database);

    return table_msgs;
  }

  static void remove_temporary_dir()
  {
#if defined(__linux__) || defined(__APPLE__)
    std::string delete_directory_command = "exec rm -r " + temporary_dir_path_;
    system(delete_directory_command.c_str());
#elif defined(_WIN32)
    // TODO(botteroa-si): find a way to delete a not empty directory in Windows, so that we don't
    // need the Fixture destructor anymore.
    RemoveDirectoryA(temporary_dir_path_.c_str())
#endif
  }

  std::string database_name_;
  static std::string temporary_dir_path_;
};

std::string Rosbag2TestFixture::temporary_dir_path_ = "";  // NOLINT

#endif  // ROSBAG2__ROSBAG2_TEST_FIXTURE_HPP_
