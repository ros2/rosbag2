// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2__READER_HPP_
#define ROSBAG2__READER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_factory_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2/converter.hpp"
#include "rosbag2/serialization_format_converter_factory.hpp"
#include "rosbag2/serialization_format_converter_factory_interface.hpp"
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
 * The Reader allows opening and reading messages of a bag. Messages will be read sequentially
 * according to the timestamp. This is not the same as a SequentialReader which reads both
 * messages and multiple database files sequentially.
 */
class ROSBAG2_PUBLIC Reader
{
public:
  // TODO(piraka9011): Re-evaluate usage of shared_ptr in converter.
  explicit Reader(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory =
    std::make_unique<rosbag2_storage::StorageFactory>(),
    std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory =
    std::make_shared<SerializationFormatConverterFactory>());

  virtual ~Reader();

  /**
   * Close the storage object. Calls the reset function of the storage implementation which
   * should close database connections.
   */
  void reset();

  /**
   * Open a rosbag for reading messages sequentially (time-ordered). Throws if file could not be
   * opened. This must be called before any other function is used. The rosbag is
   * automatically closed on destruction.
   *
   * If the `output_serialization_format` within the `converter_options` is not the same as the
   * format of the underlying stored data, a converter will be used to automatically convert the
   * data to the specified output format.
   * Throws if the converter plugin does not exist.
   *
   * \param storage_options Options to configure the storage
   * \param converter_options Options for specifying the output data format
   */
  virtual void open(
    const StorageOptions & storage_options, const ConverterOptions & converter_options);

  /**
   * Ask whether the underlying bagfile contains at least one more message.
   *
   * \return true if storage contains at least one more message
   * \throws runtime_error if the Reader is not open.
   */
  virtual bool has_next() const;

  /**
   * Read next message from storage. Will throw if no more messages are available.
   * The message will be serialized in the format given to `open`.
   *
   * Expected usage:
   * if (reader.has_next()) message = reader.read_next();
   *
   * \return next message in serialized form
   * \throws runtime_error if the Reader is not open.
   */
  virtual std::shared_ptr<SerializedBagMessage> read_next();

  /**
   * Ask bagfile for all topics (including their type identifier) that were recorded.
   *
   * \return vector of topics with topic name and type as std::string
   * \throws runtime_error if the Reader is not open.
   */
  virtual std::vector<TopicMetadata> get_all_topics_and_types() const;

  /**
   * Open a new database file to read from.
   *
   * \return true if there is a file to read from. false otherwise.
   */
  bool open_read_only(const std::string & file, const std::string & storage_id);

  /**
   * Gets a copy of the metadata file read from the storage factory.
   * TODO(piraka9011): Do we want to optimize to avoid copying?
   *
   * \return BagMetadata object with the info read from a metadata.yaml file
   */
  rosbag2_storage::BagMetadata get_metadata() const;

private:
  /**
   * Checks if all topics in the bagfile have the same RMW serialization format.
   * Currently a bag file can only be played if all topics have the same serialization format.
   *
   * \param topics Vector of TopicInformation with metadata.
   * \throws runtime_error if any topic has a different serialization format from the rest.
   */
  void check_topics_serialization_formats(const std::vector<TopicInformation> & topics);

  /**
   * Checks if the serialization format of the converter factory is the same as that of the storage factory.
   * If not, changes the serialization format of the converter factory to use the serialization format of
   * the storage factory.
   */
  void check_converter_serialization_format(
    const std::string & converter_serialization_format,
    const std::string & storage_serialization_format);

  /**
   * Ask if we have a storage object to read from.
   *
   * \return true if storage exists. false if it is null.
   */
  bool has_storage() const;
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory_;
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory_;
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage_ {nullptr};
  std::unique_ptr<Converter> converter_ {nullptr};
};

}  // namespace rosbag2

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2__READER_HPP_
