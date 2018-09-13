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

#ifndef ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_
#define ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_

#include <string>
#include <vector>

#include "rosbag2_transport/visibility_control.h"

namespace rosbag2_transport
{

class ROSBAG2_TRANSPORT_PUBLIC Rosbag2Transport
{
public:
  Rosbag2Transport() = default;

  virtual ~Rosbag2Transport() = default;

  void record(const std::vector<std::string> & topic_names);
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_
