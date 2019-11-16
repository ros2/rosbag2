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

#ifndef ROSBAG2__READERS__SEQUENTIAL_READER_HPP_
#define ROSBAG2__READERS__SEQUENTIAL_READER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_factory_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"

#include "rosbag2/converter.hpp"
#include "rosbag2/reader_interfaces/base_reader_interface.hpp"
#include "rosbag2/serialization_format_converter_factory.hpp"
#include "rosbag2/serialization_format_converter_factory_interface.hpp"
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
namespace readers
{

class ROSBAG2_PUBLIC SequentialReader : public ::rosbag2::reader_interfaces::BaseReaderInterface
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
    const StorageOptions & storage_options, const ConverterOptions & converter_options) override;

  void reset() override;

  bool has_next() override;

  std::shared_ptr<SerializedBagMessage> read_next() override;

  std::vector<TopicMetadata> get_all_topics_and_types() override;

private:
  /**
   * Checks if all topics in the bagfile have the same RMW serialization format.
   * Currently a bag file can only be played if all topics have the same serialization format.
   *
   * \param topics Vector of TopicInformation with metadata.
   * \throws runtime_error if any topic has a different serialization format from the rest.
   */
  virtual void check_topics_serialization_formats(const std::vector<TopicInformation> & topics);

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
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory_{};
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage_{};
  std::unique_ptr<Converter> converter_{};
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io_{};
  rosbag2_storage::BagMetadata metadata_{};
};

}  // namespace readers
}  // namespace rosbag2

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2__READERS__SEQUENTIAL_READER_HPP_
