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

#ifndef ROSBAG2_CPP__READERS__SEQUENTIAL_READER_HPP_
#define ROSBAG2_CPP__READERS__SEQUENTIAL_READER_HPP_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/converter.hpp"
#include "rosbag2_cpp/reader_interfaces/base_reader_interface.hpp"
#include "rosbag2_cpp/serialization_format_converter_factory.hpp"
#include "rosbag2_cpp/serialization_format_converter_factory_interface.hpp"
#include "rosbag2_cpp/visibility_control.hpp"

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_factory_interface.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_cpp
{
namespace readers
{

class ROSBAG2_CPP_PUBLIC SequentialReader
  : public ::rosbag2_cpp::reader_interfaces::BaseReaderInterface
{
public:
  SequentialReader(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory =
    std::make_unique<rosbag2_storage::StorageFactory>(),
    std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory =
    std::make_shared<SerializationFormatConverterFactory>(),
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io =
    std::make_unique<rosbag2_storage::MetadataIo>());

  virtual ~SequentialReader();

  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    const ConverterOptions & converter_options) override;

  void close() override;

  /**
   * \throws runtime_error if the Reader is not open.
   * \note Calling set_read_order(order) concurrently with has_next(), seek(t), has_next_file()
   * or load_next_file() will cause undefined behavior.
   */
  bool set_read_order(const rosbag2_storage::ReadOrder & order) override;

  bool has_next() override;

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;

  const rosbag2_storage::BagMetadata & get_metadata() const override;

  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() const override;

  void get_all_message_definitions(
    std::vector<rosbag2_storage::MessageDefinition> & definitions) override;

  void set_filter(const rosbag2_storage::StorageFilter & storage_filter) override;

  void reset_filter() override;

  /**
   * seek(t) will cause subsequent reads to return messages that satisfy
   * timestamp >= time t.
   */
  void seek(const rcutils_time_point_value_t & timestamp) override;

  /**
   * Ask whether there is another storage file to read from the list of relative file paths.
   *
   * \return true if iteration is not on the last file
   */
  virtual bool has_next_file() const;

  /**
   * Ask whether there is a previous file to read from the list of relative file paths.
   *
   * \return true if iteration is not on the first file
   */
  virtual bool has_prev_file() const;

  /**
  * Return the relative file path pointed to by the current file iterator.
  */
  virtual std::string get_current_file() const;

  /**
  * Return the URI of the current file (i.e. no extensions).
  */
  virtual std::string get_current_uri() const;

  /**
   * \brief Add callbacks for events that may occur during bag reading.
   * \param callbacks the structure containing the callback to add for each event.
   */
  void add_event_callbacks(const bag_events::ReaderEventCallbacks & callbacks) override;

  /**
   * \brief Check if a callback is registered for the given event.
   * \return True if there is any callback registered for the event, false otherwise.
   */
  bool has_callback_for_event(const bag_events::BagEvent event) const;

protected:
  /**
  * Opens the current file and sets up the filters in the new storage.
  *
  */
  virtual void load_current_file();


  /**
  * Increment the current file iterator to point to the next file in the list of relative file
  * paths, and opens that file by calling open_current_file()
  *
  * Expected usage:
  * if (has_next_file()) load_next_file();
  */
  virtual void load_next_file();

  /**
  * Increment the current file iterator to point to the previous file in the list of relative file
  * paths, and opens that file by calling open_current_file()
  *
  * Expected usage:
  * if (has_prev_file()) load_prev_file();
  */
  virtual void load_prev_file();

  /**
   * Checks if all topics in the bagfile have the same RMW serialization format.
   * Currently a bag file can only be played if all topics have the same serialization format.
   *
   * \param topics Vector of TopicInformation with metadata.
   * \throws runtime_error if any topic has a different serialization format from the rest.
   */
  virtual void check_topics_serialization_formats(
    const std::vector<rosbag2_storage::TopicInformation> & topics);

  /**
  * Checks if the serialization format of the converter factory is the same as that of the storage
  * factory.
  * If not, changes the serialization format of the converter factory to use the serialization
  * format of the storage factory.
  *
  * \param converter_serialization_format
  * \param storage_serialization_format
  */
  virtual void check_converter_serialization_format(
    const std::string & converter_serialization_format,
    const std::string & storage_serialization_format);

  /**
    * Fill topics_metadata_ cache vector with information from metadata_
    */
  virtual void fill_topics_metadata();

  /**
    * Prepare current file for opening by the storage implementation.
    * This may be used by subclasses, for example decompressing.
    * This should be a once-per-file operation, meaning that subsequent opening
    * of the same file will not trigger another preprocessing.
    */
  virtual void preprocess_current_file() {}

  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory_{};
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage_{};
  std::unique_ptr<Converter> converter_{};
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io_{};
  rosbag2_storage::BagMetadata metadata_{};
  rcutils_time_point_value_t seek_time_ = 0;
  rosbag2_storage::StorageFilter topics_filter_{};
  std::vector<rosbag2_storage::TopicMetadata> topics_metadata_{};
  std::vector<std::string> file_paths_{};
  std::vector<std::string>::iterator current_file_iterator_{};
  std::unordered_set<std::string> preprocessed_file_paths_;

  // Hang on to this because storage_options_ is mutated to point at individual files
  std::string base_folder_;

private:
  rosbag2_storage::StorageOptions storage_options_;
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory_{};

  bag_events::EventCallbackManager callback_manager_;
  rosbag2_storage::ReadOrder read_order_{};
};

}  // namespace readers
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__READERS__SEQUENTIAL_READER_HPP_
