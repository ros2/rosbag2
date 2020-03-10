// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_COMPRESSION__SEQUENTIAL_COMPRESSION_READER_HPP_
#define ROSBAG2_COMPRESSION__SEQUENTIAL_COMPRESSION_READER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_compression/base_decompressor_interface.hpp"
#include "rosbag2_compression/compression_options.hpp"

#include "rosbag2_cpp/converter.hpp"
#include "rosbag2_cpp/reader_interfaces/base_reader_interface.hpp"
#include "rosbag2_cpp/serialization_format_converter_factory.hpp"
#include "rosbag2_cpp/serialization_format_converter_factory_interface.hpp"

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_factory_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"

#include "compression_factory.hpp"
#include "visibility_control.hpp"
#include "zstd_decompressor.hpp"


#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_compression
{

class ROSBAG2_COMPRESSION_PUBLIC SequentialCompressionReader
  : public ::rosbag2_cpp::reader_interfaces::BaseReaderInterface
{
public:
  explicit SequentialCompressionReader(
    std::unique_ptr<rosbag2_compression::CompressionFactory> =
    std::make_unique<rosbag2_compression::CompressionFactory>(),
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory =
    std::make_unique<rosbag2_storage::StorageFactory>(),
    std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface> converter_factory =
    std::make_shared<rosbag2_cpp::SerializationFormatConverterFactory>(),
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io =
    std::make_unique<rosbag2_storage::MetadataIo>());

  virtual ~SequentialCompressionReader();

  void open(
    const rosbag2_cpp::StorageOptions & storage_options,
    const rosbag2_cpp::ConverterOptions & converter_options) override;

  void reset() override;

  bool has_next() override;

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;

  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() override;

  /**
   * Ask whether there is another database file to read from the list of relative
   * file paths.
   *
   * \return true if there are still files to read in the list
   */
  virtual bool has_next_file() const;

  /**
   * Return the relative file path pointed to by the current file iterator.
   */
  virtual std::string get_current_file() const;

  /**
   * Return the URI of the current file (i.e. no extensions).
   */
  virtual std::string get_current_uri() const;

protected:
  /**
   * Increment the current file iterator to point to the next file in the list of relative file
   * paths.
   *
   * Expected usage:
   * if (has_next_file()) load_next_file();
   */
  virtual void load_next_file();

  /**
   * Initializes the decompressor if a compression mode is specified in the metadata.
   *
   * \throws std::invalid_argument If compression format doesn't exist.
   */
  virtual void setup_decompression();

  /**
   * Set the file path currently pointed to by the iterator.
   *
   * \param file Relative path to the file.
   */
  virtual void set_current_file(const std::string & file);

private:
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

  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory_{};
  std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface> converter_factory_{};
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage_{};
  std::unique_ptr<rosbag2_cpp::Converter> converter_{};
  std::unique_ptr<rosbag2_compression::BaseDecompressorInterface> decompressor_{};
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io_{};
  rosbag2_storage::BagMetadata metadata_{};
  std::vector<std::string> file_paths_{};  // List of database files.
  std::vector<std::string>::iterator current_file_iterator_{};  // Index of file to read from
  rosbag2_compression::CompressionMode compression_mode_{
    rosbag2_compression::CompressionMode::NONE};
  std::unique_ptr<rosbag2_compression::CompressionFactory> compression_factory_{};
};

}  // namespace rosbag2_compression

#endif  // ROSBAG2_COMPRESSION__SEQUENTIAL_COMPRESSION_READER_HPP_
