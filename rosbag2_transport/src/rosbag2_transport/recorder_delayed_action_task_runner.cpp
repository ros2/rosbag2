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

#include "rosbag2_transport/recorder_delayed_action_task_runner.hpp"

#include <utility>

#include "recorder_delayed_action_task_runner_impl.hpp"

namespace rosbag2_transport
{

RecorderDelayedActionTaskRunner::RecorderDelayedActionTaskRunner(rclcpp::Node * node)
: pimpl_(std::make_unique<RecorderDelayedActionTaskRunnerImpl>(node))
{}

RecorderDelayedActionTaskRunner::~RecorderDelayedActionTaskRunner() = default;

void RecorderDelayedActionTaskRunner::start()
{
  pimpl_->start();
}

void RecorderDelayedActionTaskRunner::stop()
{
  pimpl_->stop();
}

void RecorderDelayedActionTaskRunner::schedule(
  const rclcpp::Time & scheduled_time,
  std::function<void()> action,
  const std::string & description)
{
  pimpl_->schedule(scheduled_time, std::move(action), description);
}

}  // namespace rosbag2_transport
