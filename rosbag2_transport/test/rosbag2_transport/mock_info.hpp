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

#ifndef ROSBAG2_TRANSPORT__MOCK_INFO_HPP_
#define ROSBAG2_TRANSPORT__MOCK_INFO_HPP_

#include <gmock/gmock.h>

#include <string>

#include "rosbag2_cpp/info.hpp"

#include "rosbag2_storage/bag_metadata.hpp"

class MockInfo : public rosbag2_cpp::Info
{
public:
  MOCK_METHOD2(
    read_metadata, rosbag2_storage::BagMetadata(const std::string &, const std::string &));
};

#endif  // ROSBAG2_TRANSPORT__MOCK_INFO_HPP_
