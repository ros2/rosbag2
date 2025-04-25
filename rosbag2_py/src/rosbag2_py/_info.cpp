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

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "info_sorting_method.hpp"
#include "format_bag_metadata.hpp"
#include "format_service_info.hpp"
#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/service_utils.hpp"
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

  void print_output(
    const rosbag2_storage::BagMetadata & metadata_info, const std::string & sorting_method = "name")
  {
    InfoSortingMethod sort_method = info_sorting_method_from_string(sorting_method);
    // Output formatted metadata
    std::cout << format_bag_meta_data(metadata_info, false, false, {}, sort_method) << std::endl;
  }

  void print_output_topic_name_only(
    const rosbag2_storage::BagMetadata & metadata_info, const std::string & sorting_method)
  {
    InfoSortingMethod sort_method = info_sorting_method_from_string(sorting_method);
    std::vector<size_t> sorted_idx = generate_sorted_idx(
      metadata_info.topics_with_message_count,
      sort_method);

    for (auto idx : sorted_idx) {
      const auto & topic_info = metadata_info.topics_with_message_count[idx];
      if (!rosbag2_cpp::is_service_event_topic(
          topic_info.topic_metadata.name,
          topic_info.topic_metadata.type))
      {
        std::cout << topic_info.topic_metadata.name << std::endl;
      }
    }
  }

  void print_output_verbose(
    const std::string & uri,
    const rosbag2_storage::BagMetadata & metadata_info,
    const std::string & sorting_method = "name")
  {
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

    std::unordered_map<std::string, uint64_t> messages_size = {};
    for (const auto & file_info : metadata_info.files) {
      auto messages_size_tmp = info_->compute_messages_size_contribution(
        uri + "/" + file_info.path,
        metadata_info.storage_identifier);
      for (const auto & topic_size_tmp : messages_size_tmp) {
        messages_size[topic_size_tmp.first] += topic_size_tmp.second;
      }
    }

    rosbag2_py::InfoSortingMethod sort_method = info_sorting_method_from_string(sorting_method);
    // Output formatted metadata and service info
    std::cout << format_bag_meta_data(metadata_info, true, true, messages_size, sort_method);
    std::cout <<
      format_service_info(all_services_info, messages_size, true, sort_method) << std::endl;
  }

  std::unordered_set<std::string> get_sorting_methods()
  {
    std::unordered_set<std::string> sorting_methods;
    for (const auto & sorting_method : rosbag2_py::sorting_method_map) {
      sorting_methods.insert(sorting_method.first);
    }
    return sorting_methods;
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
  .def("print_output", &rosbag2_py::Info::print_output)
  .def("print_output_topic_name_only", &rosbag2_py::Info::print_output_topic_name_only)
  .def("print_output_verbose", &rosbag2_py::Info::print_output_verbose)
  .def("get_sorting_methods", &rosbag2_py::Info::get_sorting_methods);
}
