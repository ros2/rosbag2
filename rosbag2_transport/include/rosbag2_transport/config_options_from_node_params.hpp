// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_TRANSPORT__CONFIG_OPTIONS_FROM_NODE_PARAMS_HPP_
#define ROSBAG2_TRANSPORT__CONFIG_OPTIONS_FROM_NODE_PARAMS_HPP_

#include <memory>

#include "rclcpp/node.hpp"
#include "rosbag2_transport/play_options.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace rosbag2_transport
{
rosbag2_transport::PlayOptions
get_play_options_from_node_params(rclcpp::Node & node);

rosbag2_transport::RecordOptions
get_record_options_from_node_params(rclcpp::Node & node);

rosbag2_storage::StorageOptions
get_storage_options_from_node_params(rclcpp::Node & node);
}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__CONFIG_OPTIONS_FROM_NODE_PARAMS_HPP_
