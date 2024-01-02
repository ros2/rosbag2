// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <memory>
#include <string>

#include "format_bag_metadata.hpp"
#include "format_service_info.hpp"
#include "rosbag2_cpp/info.hpp"
#include "rosbag2_storage/bag_metadata.hpp"

#include "pybind11.hpp"

namespace rosbag2_py
{

class Info
{
public:
  Info()
  : info_(std::make_unique<rosbag2_cpp::Info>())
  {}

  ~Info() = default;

  rosbag2_storage::BagMetadata read_metadata(
    const std::string & uri, const std::string & storage_id)
  {
    return info_->read_metadata(uri, storage_id);
  }

  void read_metadata_and_output_service_verbose(
    const std::string & uri,
    const std::string & storage_id)
  {
    auto metadata_info = read_metadata(uri, storage_id);

    std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> all_services_info;
    for (auto & file_info : metadata_info.files) {
      auto services_info = info_->read_service_info(
        uri + "/" + file_info.path,
        metadata_info.storage_identifier);
      if (!services_info.empty()) {
        all_services_info.insert(
          all_services_info.end(),
          services_info.begin(),
          services_info.end());
      }
    }

    // Output formatted metadata and service info
    std::cout << format_bag_meta_data(metadata_info, true);
    std::cout << format_service_info(all_services_info) << std::endl;
  }

protected:
  std::unique_ptr<rosbag2_cpp::Info> info_;
};

}  // namespace rosbag2_py

PYBIND11_MODULE(_info, m) {
  m.doc() = "Python wrapper of the rosbag2_cpp info API";

  pybind11::class_<rosbag2_py::Info>(m, "Info")
  .def(pybind11::init())
  .def("read_metadata", &rosbag2_py::Info::read_metadata)
  .def(
    "read_metadata_and_output_service_verbose",
    &rosbag2_py::Info::read_metadata_and_output_service_verbose);
}
