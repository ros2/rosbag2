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

#ifndef ROSBAG2__WRITER_HPP_
#define ROSBAG2__WRITER_HPP_

#include <memory>
#include <string>

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_factory_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2/converter.hpp"
#include "rosbag2/serialization_format_converter_factory.hpp"
#include "rosbag2/storage_options.hpp"
#include "rosbag2/types.hpp"
#include "rosbag2/visibility_control.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2
{

/**
 * The Writer allows writing messages to a new bag. For every topic, information about its type
 * needs to be added before writing the first message.
 */
class ROSBAG2_PUBLIC Writer
{
public:
  explicit
  Writer(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory =
    std::make_unique<rosbag2_storage::StorageFactory>(),
    std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory =
    std::make_shared<SerializationFormatConverterFactory>(),
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io =
    std::make_unique<rosbag2_storage::MetadataIo>());

  virtual ~Writer();

  /**
   * Opens a new bagfile and prepare it for writing messages. The bagfile must not exist.
   * This must be called before any other function is used.
   *
   * \param storage_options Options to configure the storage
   * \param converter_options options to define in which format incoming messages are stored
   **/
  virtual void open(
    const StorageOptions & storage_options,
    const ConverterOptions & converter_options);

  /**
   * Create a new topic in the underlying storage. Needs to be called for every topic used within
   * a message which is passed to write(...).
   *
   * \param topic_with_type name and type identifier of topic to be created
   * \throws runtime_error if the Writer is not open.
   */
  virtual void create_topic(const TopicMetadata & topic_with_type);

  /**
   * Remove a new topic in the underlying storage.
   * If creation of subscription fails remove the topic
   * from the db (more of cleanup)
   *
   * \param topic_with_type name and type identifier of topic to be created
   * \throws runtime_error if the Writer is not open.
   */
  virtual void remove_topic(const TopicMetadata & topic_with_type);

  /**
   * Write a message to a bagfile. The topic needs to have been created before writing is possible.
   *
   * \param message to be written to the bagfile
   * \throws runtime_error if the Writer is not open.
   */
  virtual void write(std::shared_ptr<SerializedBagMessage> message);

  /**
   * Check if the current recording database file needs to be split and rolled over to new file.
   */
  virtual bool should_split_database() const;

private:
  void initialize_metadata_();
  void aggregate_metadata_(rosbag2_storage::BagMetadata metadata);

  std::string uri_;
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory_;
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory_;
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage_;
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io_;
  std::unique_ptr<Converter> converter_;
  uint64_t max_bagfile_size_;
  rosbag2_storage::BagMetadata metadata_;
};

}  // namespace rosbag2

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2__WRITER_HPP_
