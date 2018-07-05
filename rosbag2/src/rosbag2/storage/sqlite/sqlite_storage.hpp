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

#ifndef ROSBAG2_SQLITESTORAGE_HPP
#define ROSBAG2_SQLITESTORAGE_HPP

#include "../storage.hpp"
#include "sqlite.hpp"

namespace rosbag2
{

class SqliteStorage : public Storage
{
public:
  SqliteStorage();
  ~SqliteStorage();

  void open(std::string filename) override;
  void insertMessage(std::string data) override;

private:
  sqlite::DBPtr database_;
};

}  // namespace rosbag2

#endif  // ROSBAG2_SQLITESTORAGE_HPP
