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

#include <cstdio>
#include <string>
#include <vector>

#ifdef _WIN32
# include <Windows.h>
#endif

#include <sqlite3.h>

using namespace ::testing;  // NOLINT

class Rosbag2TestFixture : public Test
{
public:
  Rosbag2TestFixture()
  : database_name_(UnitTest::GetInstance()->current_test_info()->name())
  {
    std::string system_separator = "/";
#ifdef _WIN32
    system_separator = "\\";
#endif
    database_name_ = temporary_dir_path_ + system_separator + database_name_;
  }

  ~Rosbag2TestFixture() override
  {
#ifdef _WIN32
    DeleteFileA(database_name_.c_str());
#else
    // TODO(botteroa-si): once filesystem::remove_all() can be used, this line can be removed and
    // the ful directory can be deleted in remove_temporary_dir()
    remove(database_name_.c_str());
#endif
  }

  static void SetUpTestCase()
  {
#ifdef _WIN32
    char dir_name[6];
    tmpnam_s(dir_name, 6);
    temporary_dir_path = std::string(dir_name);
#else
    char template_char[] = "tmp_test_dir.XXXXXX";
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

  void write_messages(std::string db_name, std::vector<std::string> messages)
  {
    sqlite3 * database;
    sqlite3_open(db_name.c_str(), &database);

    std::string create_table = "CREATE TABLE messages(" \
      "id INTEGER PRIMARY KEY AUTOINCREMENT," \
      "data           BLOB    NOT NULL," \
      "timestamp      INT     NOT NULL);";

    sqlite3_exec(database, create_table.c_str(), nullptr, nullptr, nullptr);

    for (const auto & message : messages) {
      std::string insert_message =
        "INSERT INTO messages (data, timestamp) VALUES ('" + message + "', strftime('%s%f','now'))";
      sqlite3_exec(database, insert_message.c_str(), nullptr, nullptr, nullptr);
    }
    sqlite3_close(database);
  }

  std::string database_name_;
  static std::string temporary_dir_path_;
};

std::string Rosbag2TestFixture::temporary_dir_path_ = "";  // NOLINT

#endif  // ROSBAG2__ROSBAG2_TEST_FIXTURE_HPP_
