// Copyright 2020 DCS Corporation, All Rights Reserved.
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
//
// DISTRIBUTION A. Approved for public release; distribution unlimited.
// OPSEC #4584.
//
// Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS
// Part 252.227-7013 or 7014 (Feb 2014).
//
// This notice must appear in all copies of this file and its derivatives.

#ifndef ROSBAG2_CPP__REINDEXER_HPP_
#define ROSBAG2_CPP__REINDEXER_HPP_

#include <filesystem>
#include <memory>
#include <regex>
#include <string>
#include <vector>

#include "rosbag2_cpp/converter.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/serialization_format_converter_factory.hpp"
#include "rosbag2_cpp/serialization_format_converter_factory_interface.hpp"
#include "rosbag2_cpp/visibility_control.hpp"

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_factory_interface.hpp"
#include "rosbag2_storage/storage_options.hpp"
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

/**
 * Tool to reconstruct bag metadata files in the event of loss or corruption
 *
 * Reindexing recreates metadata.yaml for a bag that is missing that file.
 * This is done by opening the storage directly and reading the contents to accumulate metadata.
 *
 * Reindexing has some limitations - It cannot perfectly replicate the original metadata file,
 *   since some information known by the program from the start up command cannot be found
 *   within the metadata. But it should at least repair a bag to the point it can be read
 *   again.
 *
 */
class ROSBAG2_CPP_PUBLIC Reindexer
{
public:
  Reindexer(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory =
    std::make_unique<rosbag2_storage::StorageFactory>(),
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io =
    std::make_unique<rosbag2_storage::MetadataIo>());

  virtual ~Reindexer() = default;

  /// Use the supplied storage options to reindex a bag defined by the storage options URI.
  /*
  * \param storage_options Provides best-guess parameters for the bag's original settings.
  */
  void reindex(const rosbag2_storage::StorageOptions & storage_options);

protected:
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory_{};
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io_{};
  rosbag2_storage::BagMetadata metadata_{};
  std::vector<rosbag2_storage::TopicMetadata> topics_metadata_{};

private:
  std::string regex_bag_pattern_;
  std::filesystem::path base_folder_;   // The folder that the bag files are in
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory_{};
  void get_bag_files(
    const std::filesystem::path & base_folder,
    std::vector<std::filesystem::path> & output);

  // Prepares the metadata by setting initial values.
  void init_metadata(
    const std::vector<std::filesystem::path> & files,
    const rosbag2_storage::StorageOptions & storage_options);

  // Attempts to harvest metadata from all bag files, and aggregates the result
  void aggregate_metadata(
    const std::vector<std::filesystem::path> & files,
    const std::unique_ptr<rosbag2_cpp::readers::SequentialReader> & bag_reader,
    const rosbag2_storage::StorageOptions & storage_options);

  // Comparison function for std::sort with our filepath convention
  bool compare_relative_file(
    const std::filesystem::path & first_path,
    const std::filesystem::path & second_path);
};

}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__REINDEXER_HPP_
