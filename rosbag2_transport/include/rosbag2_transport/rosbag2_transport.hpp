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

#ifndef ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_
#define ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_transport/rosbag2_play_options.hpp"
#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{

class GenericPublisher;
class GenericSubscription;
class Rosbag2Node;
class Player;

class Rosbag2Transport
{
public:
  ROSBAG2_TRANSPORT_PUBLIC
  Rosbag2Transport();

  ROSBAG2_TRANSPORT_PUBLIC
  void init();

  ROSBAG2_TRANSPORT_PUBLIC
  void shutdown();

  /**
   * Records topics to a bagfile. Subscription happens at startup time, hence the topics must
   * exist when "record" is called.
   *
   * \param file_name Name of the bagfile to write
   * \param topic_names Vector of topics to subscribe to. Topics must exist at startup time. If
   * the vector is empty, all topics will be subscribed.
   */
  ROSBAG2_TRANSPORT_PUBLIC
  void record(const std::string & file_name, const std::vector<std::string> & topic_names);

  /**
   * Records all available topics to a bagfile. Subscription happens at startup time, hence only
   * topics available at startup time are recorded.
   *
   * \param file_name Name of the bagfile to write
   */
  ROSBAG2_TRANSPORT_PUBLIC
  void record(const std::string & file_name);

  /**
   * Replay all topics in a bagfile.
   *
   * \param file_name Name of the bagfile to replay
   */
  ROSBAG2_TRANSPORT_PUBLIC
  void play(const std::string & file_name, const Rosbag2PlayOptions & options);

private:
  std::shared_ptr<rosbag2_transport::GenericSubscription>
  create_subscription(
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage,
    const std::string & topic_name, const std::string & topic_type) const;

  rosbag2_storage::StorageFactory factory_;
  std::shared_ptr<Rosbag2Node> node_;
  std::vector<std::shared_ptr<GenericSubscription>> subscriptions_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_
