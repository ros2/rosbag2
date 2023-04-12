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

#include "rosbag2_transport/recorder.hpp"

#include <algorithm>
#include <future>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"

#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_interfaces/srv/snapshot.hpp"

#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_transport/qos.hpp"

#include "rosbag2_transport/topic_filter.hpp"
#include "recorder_impl.hpp"

namespace rosbag2_transport
{

Recorder::Recorder(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  // TODO(karsten1987): Use this constructor later with parameter parsing.
  // The reader, storage_options as well as record_options can be loaded via parameter.
  // That way, the recorder can be used as a simple component in a component manager.
  throw rclcpp::exceptions::UnimplementedError();
}

Recorder::Recorder(
  std::shared_ptr<rosbag2_cpp::Writer> writer,
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::RecordOptions & record_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Recorder(
    std::move(writer),
#ifndef _WIN32
    std::make_shared<KeyboardHandler>(false),
#else
    // We don't have signal handler option in constructor for windows version
    std::shared_ptr<KeyboardHandler>(new KeyboardHandler()),
#endif
    storage_options,
    record_options,
    node_name,
    node_options)
{}

Recorder::Recorder(
  std::shared_ptr<rosbag2_cpp::Writer> writer,
  std::shared_ptr<KeyboardHandler> keyboard_handler,
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::RecordOptions & record_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, rclcpp::NodeOptions(node_options)
    .start_parameter_event_publisher(false)
    .parameter_overrides({rclcpp::Parameter("use_sim_time", record_options.use_sim_time)})),
  impl_(std::make_unique<RecorderImpl>(
      writer,
      keyboard_handler,
      storage_options,
      record_options,
      this,
      kPauseResumeToggleKey
    )) {}


Recorder::~Recorder() {}

void Recorder::record()
{
  impl_->record();
}

const rosbag2_cpp::Writer & Recorder::get_writer_handle()
{
  return impl_->get_writer_handle();
}

void Recorder::pause()
{
  return impl_->pause();
}

void Recorder::resume()
{
  return impl_->resume();
}

void Recorder::toggle_paused()
{
  return impl_->toggle_paused();
}

bool Recorder::is_paused()
{
  return impl_->is_paused();
}
std::unordered_map<std::string, std::string>
Recorder::get_requested_or_available_topics()
{
  return impl_->get_requested_or_available_topics();
}

}  // namespace rosbag2_transport
