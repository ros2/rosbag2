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

#ifndef ROSBAG2_CPP__WRITER_HPP_
#define ROSBAG2_CPP__WRITER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/time.hpp"

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_cpp
{
namespace writer_interfaces
{
class BaseWriterInterface;
}  // namespace writer_interfaces

/**
 * The Writer allows writing messages to a new bag. For every topic, information about its type
 * needs to be added before writing the first message.
 */
class ROSBAG2_CPP_PUBLIC Writer final
{
public:
  explicit Writer(
    std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> writer_impl =
    std::make_unique<writers::SequentialWriter>());

  ~Writer();

  /**
   * Opens a new bagfile and prepare it for writing messages. The bagfile must not exist.
   * This must be called before any other function is used.
   *
   * \note This will open URI with the default storage options
   * * using sqlite3 storage backend
   * * using no converter options, storing messages with the incoming serialization format
   * \sa rmw_get_serialization_format.
   * For specifications, please see \sa open, which let's you specify
   * more storage and converter options.
   *
   * \param storage_uri URI of the storage to open.
   **/
  void open(const std::string & uri);

  /**
   * Opens a new bagfile and prepare it for writing messages. The bagfile must not exist.
   * This must be called before any other function is used.
   *
   * \param storage_options Options to configure the storage
   * \param converter_options options to define in which format incoming messages are stored
   **/
  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    const ConverterOptions & converter_options = ConverterOptions());

  /**
   * Create a new topic in the underlying storage. Needs to be called for every topic used within
   * a message which is passed to write(...).
   *
   * \param topic_with_type name and type identifier of topic to be created
   * \throws runtime_error if the Writer is not open.
   */
  void create_topic(const rosbag2_storage::TopicMetadata & topic_with_type);

  /**
   * Remove a new topic in the underlying storage.
   * If creation of subscription fails remove the topic
   * from the db (more of cleanup)
   *
   * \param topic_with_type name and type identifier of topic to be created
   * \throws runtime_error if the Writer is not open.
   */
  void remove_topic(const rosbag2_storage::TopicMetadata & topic_with_type);

  /**
   * Write a message to a bagfile. The topic needs to have been created before writing is possible.
   *
   * \param message to be written to the bagfile
   * \throws runtime_error if the Writer is not open.
   */
  void write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message);

  /**
   * Write a message to a bagfile.
   * The topic will be created if it has not been created already.
   *
   * \param message to be written to the bagfile
   * \param topic_name the string of the topic this messages belongs to
   * \param type_name the string of the type associated with this message
   * \param serialization_format the format in which this message is serialized
   * \throws runtime_error if the Writer is not open.
   */
  void write(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> message,
    const std::string & topic_name,
    const std::string & type_name,
    const std::string & serialization_format = "cdr");

  /**
   * Write a serialized message to a bagfile.
   * The topic will be created if it has not been created already.
   *
   * \param message rclcpp::SerializedMessage The serialized message to be written to the bagfile
   * \param topic_name the string of the topic this messages belongs to
   * \param type_name the string of the type associated with this message
   * \param time The time stamp of the message
   * \throws runtime_error if the Writer is not open.
   */
  void write(
    const rclcpp::SerializedMessage & message,
    const std::string & topic_name,
    const std::string & type_name,
    const rclcpp::Time & time);

  /**
   * Write a non-serialized message to a bagfile.
   * The topic will be created if it has not been created already.
   *
   * \param message MessageT The serialized message to be written to the bagfile
   * \param topic_name the string of the topic this messages belongs to
   * \param type_name the string of the type associated with this message
   * \param time The time stamp of the message
   * \throws runtime_error if the Writer is not open.
   */
  template<class MessageT>
  void write(
    const MessageT & message,
    const std::string & topic_name,
    const rclcpp::Time & time)
  {
    rclcpp::SerializedMessage serialized_msg;

    rclcpp::Serialization<MessageT> serialization;
    serialization.serialize_message(&message, &serialized_msg);
    return write(serialized_msg, topic_name, rosidl_generator_traits::name<MessageT>(), time);
  }

  writer_interfaces::BaseWriterInterface & get_implementation_handle() const
  {
    return *writer_impl_;
  }

private:
  std::mutex writer_mutex_;
  std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> writer_impl_;
};

}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__WRITER_HPP_
