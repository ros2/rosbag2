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

#ifndef ROSBAG2_STORAGE__STORAGE_INTERFACES__BASE_WRITE_INTERFACE_HPP_
#define ROSBAG2_STORAGE__STORAGE_INTERFACES__BASE_WRITE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/message_definition.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_storage/visibility_control.hpp"

namespace rosbag2_storage
{

using SerializedBagMessages = std::vector<std::shared_ptr<const SerializedBagMessage>>;

namespace storage_interfaces
{

/// \brief The base interface for writing messages to storage.
class ROSBAG2_STORAGE_PUBLIC BaseWriteInterface
{
public:
  /// \brief Default destructor.
  virtual ~BaseWriteInterface() = default;

  /// \brief Writes one serialized message to the storage.
  /// \throws std::runtime_error if the storage is not open, or if create_topic(..) was not called
  /// previously for the topic associated with the message being written.
  [[deprecated("Use write_message(std::shared_ptr<const SerializedBagMessage> msg) instead.")]]
  virtual void write(std::shared_ptr<const SerializedBagMessage> msg) = 0;

  // This method is deprecated, use
  // std::vector<size_t> write_messages(const SerializedBagMessages & messages) instead.
  [[deprecated("Use write_messages(const SerializedBagMessages & messages) instead.")]]
  virtual void write(const std::vector<std::shared_ptr<const SerializedBagMessage>> & msg) = 0;

  /// \brief Writes one serialized message to the storage.
  /// \param msg - The serialized message to write.
  /// \return Returns true if the message was written successfully, false otherwise.
  /// \throws std::runtime_error if the storage is not open, or if create_topic(..) was not called
  /// previously for the topic associated with the message being written.
  virtual bool write_message(std::shared_ptr<const SerializedBagMessage> msg) = 0;

  /// \brief Write a batch of messages to the storage.
  /// \param messages - List of serialized messages to write.
  /// \return Returns a vector of indexes pointing to the messages from the input list that were
  /// not written. If all messages were written successfully, the vector will be empty.
  /// \throws std::runtime_error if the storage is not open or if create_topic(..) has not been
  /// called for all topics in the messages list.
  virtual std::vector<size_t> write_messages(const SerializedBagMessages & messages) = 0;

  /// \brief Writes provided metadata to the storage i.e. bag file.
  /// \param bag_metadata - Metadata to be written to the storage.
  virtual void update_metadata(const BagMetadata & bag_metadata) = 0;

  /// \brief Creates a new topic in the storage. Needs to be called for every topic used within a
  //// message which is passed to write_message(...) or write_messages(...).
  /// \param topic - Metadata of the topic to create.
  /// \param message_definition - rosbag2_storage::MessageDefinition containing the full encoded
  /// message definition for this type and the type hash.
  /// \throws std::runtime_error if the storage is not open.
  virtual void create_topic(
    const TopicMetadata & topic,
    const rosbag2_storage::MessageDefinition & message_definition) = 0;

  /// \brief Removes a topic from the internal storage references.
  /// This does not remove the topic from the bag file, but removes it from the internal
  /// references so that it is not written to the bag file in the future. This is an opposite to
  /// create_topic, which adds a topic to the internal storage references.
  /// \param topic - Metadata of the topic to remove.
  virtual void remove_topic(const TopicMetadata & topic) = 0;
};

}  // namespace storage_interfaces
}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__STORAGE_INTERFACES__BASE_WRITE_INTERFACE_HPP_
