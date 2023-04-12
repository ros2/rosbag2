// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_TRANSPORT__RECORDER_HPP_
#define ROSBAG2_TRANSPORT__RECORDER_HPP_

#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "keyboard_handler/keyboard_handler.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/snapshot.hpp"
#include "rosbag2_interfaces/srv/split_bagfile.hpp"

#include "rosbag2_interfaces/msg/write_split_event.hpp"

#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/visibility_control.hpp"
#include "rosbag2_transport/topic_filter.hpp"

namespace rosbag2_cpp
{
class Writer;
}

namespace rosbag2_transport
{

class RecorderImpl;

class Recorder : public rclcpp::Node
{
public:
  ROSBAG2_TRANSPORT_PUBLIC
  explicit Recorder(
    const std::string & node_name = "rosbag2_recorder",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  Recorder(
    std::shared_ptr<rosbag2_cpp::Writer> writer,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::RecordOptions & record_options,
    const std::string & node_name = "rosbag2_recorder",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  Recorder(
    std::shared_ptr<rosbag2_cpp::Writer> writer,
    std::shared_ptr<KeyboardHandler> keyboard_handler,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::RecordOptions & record_options,
    const std::string & node_name = "rosbag2_recorder",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  virtual ~Recorder();

  ROSBAG2_TRANSPORT_PUBLIC
  void record();

  ROSBAG2_TRANSPORT_PUBLIC
  const rosbag2_cpp::Writer & get_writer_handle();

  /// Pause the recording.
  ROSBAG2_TRANSPORT_PUBLIC
  void pause();

  /// Resume recording.
  ROSBAG2_TRANSPORT_PUBLIC
  void resume();

  /// Pause if it was recording, continue recording if paused.
  ROSBAG2_TRANSPORT_PUBLIC
  void toggle_paused();

  /// Return the current paused state.
  ROSBAG2_TRANSPORT_PUBLIC
  bool is_paused();

  inline constexpr static const auto kPauseResumeToggleKey = KeyboardHandler::KeyCode::SPACE;

protected:
  ROSBAG2_TRANSPORT_EXPORT
  std::unordered_map<std::string, std::string> get_requested_or_available_topics();

private:
  std::unique_ptr<RecorderImpl> impl_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__RECORDER_HPP_
