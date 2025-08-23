// Copyright 2025 Apex.AI, Inc.
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

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

#include "rosbag2_transport/recorder_event_notifier.hpp"
#include "recorder_event_notifier_impl.hpp"

namespace rosbag2_transport
{

RecorderEventNotifier::RecorderEventNotifier(rclcpp::Node * node)
{
  pimpl_ = std::make_unique<RecorderEventNotifierImpl>(node);
}

RecorderEventNotifier::~RecorderEventNotifier()
{
  // Explicitly reset the pimpl_ to ensure the destructor of RecorderEventNotifierImpl is called
  // only once.
  pimpl_.reset();
}

void RecorderEventNotifier::set_messages_lost_statistics_max_publishing_rate(float update_rate_hz)
{
  pimpl_->set_messages_lost_statistics_max_publishing_rate(update_rate_hz);
}

void RecorderEventNotifier::on_bag_split_in_recorder(
  const rosbag2_cpp::bag_events::BagSplitInfo & bag_split_info)
{
  pimpl_->on_bag_split_in_recorder(bag_split_info);
}

void RecorderEventNotifier::on_messages_lost_in_transport(
  const std::string & topic_name,
  const rclcpp::QOSMessageLostInfo & qos_msgs_lost_info)
{
  pimpl_->on_messages_lost_in_transport(topic_name, qos_msgs_lost_info);
}

uint64_t RecorderEventNotifier::get_total_num_messages_lost_in_transport() const
{
  return pimpl_->get_total_num_messages_lost_in_transport();
}

void RecorderEventNotifier::reset_total_num_messages_lost_in_transport()
{
  pimpl_->reset_total_num_messages_lost_in_transport();
}

}  // namespace rosbag2_transport
