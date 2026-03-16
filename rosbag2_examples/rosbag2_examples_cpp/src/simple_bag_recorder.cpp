// Copyright 2026 Open Source Robotics Foundation
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

#include "rosbag2_transport/recorder.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = "my_bag";

  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = true;
  record_options.rmw_serialization_format = "cdr";

  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer), storage_options, record_options);

  recorder->record();
  rclcpp::spin(recorder);

  recorder->stop();
  rclcpp::shutdown();
  return 0;
}
