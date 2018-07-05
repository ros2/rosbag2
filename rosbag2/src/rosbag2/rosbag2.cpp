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

#include <sqlite3.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

int main(int argc, const char** argv)
{
  sqlite3 *db;
  char *zErrMsg = 0;
  int rc;

  rc = sqlite3_open("test.db", &db);

  if(rc) {
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    return(0);
  } else {
    fprintf(stderr, "Opened database successfully\n");
  }

  std::string create_table = "CREATE TABLE IF NOT EXISTS messages("  \
         "id INTEGER PRIMARY KEY AUTOINCREMENT," \
         "data           BLOB    NOT NULL," \
         "timestamp      INT     NOT NULL);";

  rc = sqlite3_exec(db, create_table.c_str(), nullptr, nullptr, &zErrMsg);

  if( rc != SQLITE_OK ){
    fprintf(stderr, "SQL error: %s\n", zErrMsg);
    sqlite3_free(zErrMsg);
  } else {
    fprintf(stdout, "Table created successfully\n");
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rosbag_node");
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "string_topic",
    [db, &rc](std_msgs::msg::String::ConstSharedPtr msg) {
      char * zErrMsg2;
      std::cout << msg->data << std::endl;
      std::string insert_msg_statement =
        "INSERT INTO messages (data, timestamp) VALUES ('" + msg->data + "', "
          "strftime('%s%f','now'))";
      rc = sqlite3_exec(db, insert_msg_statement.c_str(), nullptr, nullptr, &zErrMsg2);
      if(rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg2);
        sqlite3_free(zErrMsg2);
      } else {
        fprintf(stdout, "Message inserted successfully\n");
        sqlite3_free(zErrMsg2);
      }
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  sqlite3_close(db);
  return 0;
}



