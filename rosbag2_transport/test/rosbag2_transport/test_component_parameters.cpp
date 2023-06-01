// Copyright 2023 Patrick Roncagliolo
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

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_transport_test_fixture.hpp"
#include "rosbag2_transport/recorder.hpp"

class ComposableRecorder : public rosbag2_transport::Recorder
{
public:
  bool demo_attribute;

  explicit ComposableRecorder(const rclcpp::NodeOptions & options)
  : rosbag2_transport::Recorder(
      std::make_shared<rosbag2_cpp::Writer>(),
      rosbag2_storage::StorageOptions(),
      rosbag2_transport::RecordOptions(),
      "test_recorder_component",
      options)
  {
    demo_attribute = declare_parameter<bool>("demo_attribute", false);
  }
};

// See github.com/ros2/rclcpp/blob/f8072f/rclcpp_components/src/component_manager.cpp
// at lines 137-166 - Component Manager passes params as "parameter overrides".
TEST_F(Rosbag2TransportTestFixture, params_passed_as_overrides_like_component_manager_does)
{
  rclcpp::init(0, nullptr);
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("demo_attribute", true));
  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  auto recorder = std::make_shared<ComposableRecorder>(options);
  ASSERT_TRUE(recorder->demo_attribute);
  rclcpp::shutdown();
}
