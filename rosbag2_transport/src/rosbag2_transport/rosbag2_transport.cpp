// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2020, TNG Technology Consulting GmbH.
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

#include "rosbag2_transport/rosbag2_transport.hpp"

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rcpputils/scope_exit.hpp"
#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

#include "player.hpp"
#include "recorder.hpp"

namespace
{
std::shared_ptr<rclcpp::Node> setup_node(
  std::string node_prefix = "",
  const std::vector<std::string> & topic_remapping_options = {})
{
  auto node_options = rclcpp::NodeOptions().arguments(topic_remapping_options);
  return std::make_shared<rclcpp::Node>(node_prefix + "_rosbag2", node_options);
}
}  // namespace

namespace rosbag2_transport
{

Player::Player(std::shared_ptr<rosbag2_cpp::Reader> reader)
: reader_(std::move(reader))
{}

Player::Player()
: Player(std::make_shared<rosbag2_cpp::Reader>(
      std::make_unique<rosbag2_cpp::readers::SequentialReader>()))
{}

Player::~Player()
{}

Recorder::Recorder(std::shared_ptr<rosbag2_cpp::Writer> writer)
: writer_(std::move(writer))
{}

Recorder::Recorder()
: Recorder(std::make_shared<rosbag2_cpp::Writer>(
      std::make_unique<rosbag2_cpp::writers::SequentialWriter>()))
{}

Recorder::~Recorder()
{}

void Recorder::record(
  const rosbag2_storage::StorageOptions & storage_options, const RecordOptions & record_options)
{
  try {
    writer_->open(
      storage_options, {rmw_get_serialization_format(), record_options.rmw_serialization_format});

    auto transport_node = setup_node(record_options.node_prefix);

    impl::Recorder recorder(writer_, transport_node);
    recorder.record(record_options);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rosbag2_transport"), "Failed to record: %s", e.what());
  }
}

void Player::play(
  const rosbag2_storage::StorageOptions & storage_options, const PlayOptions & play_options)
{
  auto transport_node =
    setup_node(play_options.node_prefix, play_options.topic_remapping_options);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(transport_node);
  auto spin_thread = std::thread(
    [&exec]() {
      exec.spin();
    });
  auto exit = rcpputils::scope_exit(
    [&exec, &spin_thread]() {
      exec.cancel();
      spin_thread.join();
    });
  try {
    impl::Player player(reader_, transport_node);
    do {
      reader_->open(storage_options, {"", rmw_get_serialization_format()});
      player.play(play_options);
    } while (rclcpp::ok() && play_options.loop);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(transport_node->get_logger(), "Failed to play: %s", e.what());
  }
}

}  // namespace rosbag2_transport
