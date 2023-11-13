// Copyright 2018, Patrick Roncagliolo
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

#ifndef ROSBAG2_TRANSPORT__ROSBAG2_RECORD_TEST_FIXTURE_HPP_
#define ROSBAG2_TRANSPORT__ROSBAG2_RECORD_TEST_FIXTURE_HPP_

#include <memory>

#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_transport_test_fixture.hpp"

class RosBag2RecordTestFixture : public Rosbag2TransportTestFixture
{
public:
  RosBag2RecordTestFixture()
  : Rosbag2TransportTestFixture()
  {
    rclcpp::init(0, nullptr);
    pub_ = std::make_shared<PublicationManager>();
  }

  ~RosBag2RecordTestFixture() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<PublicationManager> pub_;
};

#endif  //  ROSBAG2_TRANSPORT__ROSBAG2_RECORD_TEST_FIXTURE_HPP_
