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

namespace rosbag2_storage
{

// TODO(greimela): Move to common place (is also used in test fixture etc.)
std::string separator()
{
#ifdef _WIN32
  return "\\";
#else
  return "/";
#endif
}

MetadataIO::MetadataIO(const std::string & uri)
: file_name_(get_metadata_file_name(uri)) {
}

void MetadataIO::write_metadata(BagMetadata metadata)
{
  YAML::Node metadata_node;
  metadata_node["rosbag2_bagfile_information"] = metadata;
  std::ofstream fout(file_name_);
  fout << metadata_node;
}

BagMetadata MetadataIO::read_metadata()
{
  YAML::Node yaml_file = YAML::LoadFile(file_name_);
  auto metadata = yaml_file["rosbag2_bagfile_information"].as<rosbag2_storage::BagMetadata>();
  return metadata;
}

std::string MetadataIO::get_metadata_file_name(const std::string & uri)
{
  // TODO(botteroa-si): use metadata_file = uri + separator() + "metadata.yaml" once the uri is the
  // path to the bag directory.
  std::string metadata_file = uri + ".metadata.yaml";
  
  return metadata_file;
}

}  // namespace rosbag2_storage
