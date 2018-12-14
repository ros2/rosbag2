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

#ifndef ROSBAG2__INFO_HPP_
#define ROSBAG2__INFO_HPP_

#include <string>

#include "rosbag2/types.hpp"
#include "visibility_control.hpp"

namespace rosbag2
{

class ROSBAG2_PUBLIC Info
{
public:
  virtual ~Info() = default;

  virtual rosbag2::BagMetadata read_metadata(
    const std::string & uri, const std::string & storage_id);
};

}  // namespace rosbag2

#endif  // ROSBAG2__INFO_HPP_
