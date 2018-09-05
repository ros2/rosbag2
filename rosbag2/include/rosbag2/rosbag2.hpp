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
  /**
   * Records topics to a bagfile. Subscription happens at startup time, hence the topics must
   * exist when "record" is called.
   *
   * @param file_name Name of the bagfile to write
   * @param topic_names Vector of topics to subscribe to. Topics must exist at startup time. If
   * the vector is empty, all topics will be subscribed.
   * @param after_write_action This function will be executed after each write to the database
   * where the input parameter is the topic name of the topic written Currently needed for testing.
   * Might be removed later.
   */
  ROSBAG2_PUBLIC
  void record(
    const std::string & file_name,
    const std::vector<std::string> & topic_names,
    std::function<void(std::string)> after_write_action = nullptr);

  /**
   * Replay all topics in a bagfile.
   *
   * @param file_name Name of the bagfile to replay
   */
  ROSBAG2_PUBLIC
  void play(const std::string & file_name);

private:
  void prepare_publishers(
    std::shared_ptr<Rosbag2Node> node,
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage);

  std::shared_ptr<rosbag2::GenericSubscription>
  create_subscription(
    const std::function<void(std::string)> & after_write_action,
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage,
    std::shared_ptr<Rosbag2Node> & node,
    const std::string & topic_name, const std::string & topic_type) const;

  std::vector<std::shared_ptr<GenericSubscription>> subscriptions_;
  std::map<std::string, std::shared_ptr<GenericPublisher>> publishers_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__ROSBAG2_HPP_
