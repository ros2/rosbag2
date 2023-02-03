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

#include "rosbag2_storage/metadata_io.hpp"

#include <fstream>
#include <string>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rcutils/filesystem.h"

#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_storage/yaml.hpp"

namespace rosbag2_storage
{

void MetadataIo::write_metadata(const std::string & uri, const BagMetadata & metadata)
{
  YAML::Node metadata_node;
  metadata_node["rosbag2_bagfile_information"] = metadata;
  std::ofstream fout(get_metadata_file_name(uri));
  fout << metadata_node;
}

BagMetadata MetadataIo::read_metadata(const std::string & uri)
{
  try {
    YAML::Node yaml_file = YAML::LoadFile(get_metadata_file_name(uri));
    auto metadata = yaml_file["rosbag2_bagfile_information"].as<rosbag2_storage::BagMetadata>();
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    if (RCUTILS_RET_OK !=
      rcutils_calculate_directory_size(uri.c_str(), &metadata.bag_size, allocator))
    {
      throw std::runtime_error(
              std::string("Exception on calculating the size of directory :") + uri);
    }
    return metadata;
  } catch (const YAML::Exception & ex) {
    throw std::runtime_error(std::string("Exception on parsing info file: ") + ex.what());
  }
}

std::string MetadataIo::get_metadata_file_name(const std::string & uri)
{
  std::string metadata_file = (rcpputils::fs::path(uri) / metadata_filename).string();

  return metadata_file;
}

bool MetadataIo::metadata_file_exists(const std::string & uri)
{
  return rcpputils::fs::exists(rcpputils::fs::path(get_metadata_file_name(uri)));
}

std::string MetadataIo::serialize_metadata(const BagMetadata & metadata)
{
  auto node = YAML::convert<BagMetadata>().encode(metadata);
  std::stringstream out;
  out << node;
  return out.str();
}

BagMetadata MetadataIo::deserialize_metadata(const std::string & serialized_metadata)
{
  YAML::Node yaml = YAML::Load(serialized_metadata);
  return yaml.as<BagMetadata>();
}

}  // namespace rosbag2_storage
