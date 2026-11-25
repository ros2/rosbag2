// Copyright 2025 Open Source Robotics Foundation, Inc.
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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef ROSBAG2__COMPOSABLE_RECORDER_HPP_
#define ROSBAG2__COMPOSABLE_RECORDER_HPP_

#include <string>

#include "rclcpp/node_options.hpp"
#include "rosbag2/visibility_control.hpp"
#include "rosbag2_transport/recorder.hpp"

namespace rosbag2
{
class Recorder : public rosbag2_transport::Recorder {
public:
  /// \brief Constructor and entry point for the composable recorder.
  /// Will call Recorder(node_name, node_options) constructor with node_name = "rosbag2_recorder".
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_PUBLIC
  explicit Recorder(const rclcpp::NodeOptions & node_options);

  /// \brief Default constructor and entry point for the composable recorder.
  /// Will construct Recorder class and initialize play_options, storage_options from node
  /// parameters. At the end will call Recorder::play() to automatically start playback in a
  /// separate thread.
  /// \param node_name Name for the underlying node.
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_PUBLIC
  explicit Recorder(
    const std::string & node_name = "rosbag2_recorder",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
};
}  // namespace rosbag2

#endif  // ROSBAG2__COMPOSABLE_RECORDER_HPP_
