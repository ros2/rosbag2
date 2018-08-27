// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2__ROSBAG2_HPP_
#define ROSBAG2__ROSBAG2_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2/visibility_control.hpp"

namespace rosbag2
{

class GenericPublisher;
class GenericSubscription;
class Rosbag2Node;

class Rosbag2
{
public:
  ROSBAG2_PUBLIC
  void record(
    const std::string & file_name,
    std::vector<std::string> topic_names,
    std::function<void(void)> after_write_action = nullptr);

  ROSBAG2_PUBLIC
  void play(const std::string & file_name);

  ROSBAG2_PUBLIC
  std::map<std::string, std::string> get_topics_with_types(
    std::vector<std::string> topic_names, const std::shared_ptr<rclcpp::Node> & node);

private:
  void prepare_publishers(
    std::shared_ptr<Rosbag2Node> node,
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage);

  std::shared_ptr<rosbag2::GenericSubscription>
  create_subscription(
    const std::function<void()> & after_write_action,
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage,
    std::shared_ptr<Rosbag2Node> & node,
    const std::string & topic_name, const std::string & topic_type) const;

  std::vector<std::shared_ptr<GenericSubscription>> subscriptions_;
  std::map<std::string, std::shared_ptr<GenericPublisher>> publishers_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__ROSBAG2_HPP_
