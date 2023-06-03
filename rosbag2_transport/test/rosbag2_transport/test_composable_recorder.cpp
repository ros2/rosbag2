// Copyright 2023 Patrick Roncagliolo and Michael Orlov
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
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_transport_test_fixture.hpp"
#include "rosbag2_transport/recorder.hpp"

class ComposableRecorderTestFixture : public Rosbag2TransportTestFixture
{
public:
  ComposableRecorderTestFixture()
  : Rosbag2TransportTestFixture()
  {
    rclcpp::init(0, nullptr);
  }

  ~ComposableRecorderTestFixture() override
  {
    rclcpp::shutdown();
  }
};

class ComposableRecorder : public rosbag2_transport::Recorder
{
public:
  static const char demo_attribute_name_[];
  bool demo_attribute_value{false};

  explicit ComposableRecorder(const rclcpp::NodeOptions & options)
  : rosbag2_transport::Recorder(
      std::make_shared<rosbag2_cpp::Writer>(),
      rosbag2_storage::StorageOptions(),
      rosbag2_transport::RecordOptions(),
      "test_recorder_component",
      options)
  {
    // Declare demo attribute parameter for the underlying node with default value equal to false.
    // However, if node was created with option to override this parameter it will be settled up
    // to what was specified in parameter_overrides value.
    demo_attribute_value = this->declare_parameter<bool>(
      demo_attribute_name_, /*default_value=*/ false,
      rcl_interfaces::msg::ParameterDescriptor(), /*ignore_override=*/ false);
  }

  bool get_value_of_bool_parameter(const std::string & parameter_name)
  {
    bool ret_value{false};
    bool parameter_was_set = this->get_parameter(parameter_name, ret_value);
    if (!parameter_was_set) {
      throw std::runtime_error("Parameter `" + parameter_name + "` hasn't been set.");
    }
    return ret_value;
  }
};
const char ComposableRecorder::demo_attribute_name_[] = "demo_attribute";

TEST_F(ComposableRecorderTestFixture, recorder_inner_params_passed_as_append_override)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back(ComposableRecorder::demo_attribute_name_, true);
  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  auto recorder = std::make_shared<ComposableRecorder>(options);
  // Check that rosbag2_transport::Recorder inner params will not erase our
  // parameter_overrides options
  ASSERT_TRUE(recorder->get_value_of_bool_parameter(recorder->demo_attribute_name_));
  ASSERT_TRUE(recorder->demo_attribute_value);
}
