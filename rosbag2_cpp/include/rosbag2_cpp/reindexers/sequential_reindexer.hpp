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

#ifndef ROSBAG2_CPP__REINDEXERS__SEQUENTIAL_REINDEXER_HPP_
#define ROSBAG2_CPP__REINDEXERS__SEQUENTIAL_REINDEXER_HPP_

#include <memory>
#include <regex>
#include <string>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_cpp/converter.hpp"
#include "rosbag2_cpp/reindexer_interfaces/base_reindexer_interface.hpp"
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
namespace reindexers
{

class ROSBAG2_CPP_PUBLIC SequentialReindexer
  : public ::rosbag2_cpp::reindexer_interfaces::BaseReindexerInterface
{
public:
  SequentialReindexer(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory =
    std::make_unique<rosbag2_storage::StorageFactory>(),
    std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory =
    std::make_shared<SerializationFormatConverterFactory>(),
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io =
    std::make_unique<rosbag2_storage::MetadataIo>());

  virtual ~SequentialReindexer();


  void reindex(const StorageOptions & storage_options) override;

  void fill_topics_metadata();

  void reset();

  void finalize_metadata();

protected:
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory_{};
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage_{};
  std::unique_ptr<Converter> converter_{};
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io_{};
  rosbag2_storage::BagMetadata metadata_{};
  std::vector<rosbag2_storage::TopicMetadata> topics_metadata_{};
  std::vector<rcpputils::fs::path> file_paths_{};  // List of database files.
  // Index of file to read from
  std::vector<rcpputils::fs::path>::iterator current_file_iterator_{};

private:
  rcpputils::fs::path base_folder_;
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory_{};

  std::vector<rcpputils::fs::path> get_database_files(const rcpputils::fs::path & base_folder);

  void open(
    const rcpputils::fs::path & database_file,
    const StorageOptions & storage_options);

  // Prepares the metadata by setting initial values.
  void init_metadata(const std::vector<rcpputils::fs::path> & files);

  // Attempts to harvest metadata from all bag files, and aggregates the result
  void aggregate_metadata(
    const std::vector<rcpputils::fs::path> & files, const StorageOptions & storage_options);

  // Compairson function for std::sort with our filepath convention
  static bool comp_rel_file(
    const rcpputils::fs::path & first_path,
    const rcpputils::fs::path & second_path);
};

}  // namespace reindexers
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__REINDEXERS__SEQUENTIAL_REINDEXER_HPP_
