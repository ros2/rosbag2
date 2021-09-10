// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <string>
#include <vector>

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "./format_bag_metadata.hpp"

#include "./pybind11.hpp"

PYBIND11_MODULE(_storage, m) {
  m.doc() = "Python wrapper of the rosbag2 utilities API";

  pybind11::class_<rosbag2_cpp::ConverterOptions>(m, "ConverterOptions")
  .def(
    pybind11::init<std::string, std::string>(),
    pybind11::arg("input_serialization_format"),
    pybind11::arg("output_serialization_format"))
  .def_readwrite(
    "input_serialization_format",
    &rosbag2_cpp::ConverterOptions::input_serialization_format)
  .def_readwrite(
    "output_serialization_format",
    &rosbag2_cpp::ConverterOptions::output_serialization_format);

  pybind11::class_<rosbag2_storage::StorageOptions>(m, "StorageOptions")
  .def(
    pybind11::init<
      std::string, std::string, uint64_t, uint64_t, uint64_t, std::string, std::string>(),
    pybind11::arg("uri"),
    pybind11::arg("storage_id"),
    pybind11::arg("max_bagfile_size") = 0,
    pybind11::arg("max_bagfile_duration") = 0,
    pybind11::arg("max_cache_size") = 0,
    pybind11::arg("storage_preset_profile") = "",
    pybind11::arg("storage_config_uri") = "")
  .def_readwrite("uri", &rosbag2_storage::StorageOptions::uri)
  .def_readwrite("storage_id", &rosbag2_storage::StorageOptions::storage_id)
  .def_readwrite(
    "max_bagfile_size",
    &rosbag2_storage::StorageOptions::max_bagfile_size)
  .def_readwrite(
    "max_bagfile_duration",
    &rosbag2_storage::StorageOptions::max_bagfile_duration)
  .def_readwrite(
    "max_cache_size",
    &rosbag2_storage::StorageOptions::max_cache_size)
  .def_readwrite(
    "storage_preset_profile",
    &rosbag2_storage::StorageOptions::storage_preset_profile)
  .def_readwrite(
    "storage_config_uri",
    &rosbag2_storage::StorageOptions::storage_config_uri);

  pybind11::class_<rosbag2_storage::StorageFilter>(m, "StorageFilter")
  .def(
    pybind11::init<std::vector<std::string>>(),
    pybind11::arg("topics"))
  .def_readwrite("topics", &rosbag2_storage::StorageFilter::topics);

  pybind11::class_<rosbag2_storage::TopicMetadata>(m, "TopicMetadata")
  .def(
    pybind11::init<std::string, std::string, std::string, std::string>(),
    pybind11::arg("name"),
    pybind11::arg("type"),
    pybind11::arg("serialization_format"),
    pybind11::arg("offered_qos_profiles") = "")
  .def_readwrite("name", &rosbag2_storage::TopicMetadata::name)
  .def_readwrite("type", &rosbag2_storage::TopicMetadata::type)
  .def_readwrite(
    "serialization_format",
    &rosbag2_storage::TopicMetadata::serialization_format)
  .def_readwrite(
    "offered_qos_profiles",
    &rosbag2_storage::TopicMetadata::offered_qos_profiles)
  .def("equals", &rosbag2_storage::TopicMetadata::operator==);

  pybind11::class_<rosbag2_storage::TopicInformation>(m, "TopicInformation")
  .def(
    pybind11::init<rosbag2_storage::TopicMetadata, size_t>(),
    pybind11::arg("topic_metadata"),
    pybind11::arg("message_count"))
  .def_readwrite("topic_metadata", &rosbag2_storage::TopicInformation::topic_metadata)
  .def_readwrite("message_count", &rosbag2_storage::TopicInformation::message_count);

  pybind11::class_<rosbag2_storage::FileInformation>(m, "FileInformation")
  .def(
    pybind11::init<std::string,
    std::chrono::time_point<std::chrono::high_resolution_clock>,
    std::chrono::nanoseconds,
    size_t>(),
    pybind11::arg("path"),
    pybind11::arg("starting_time"),
    pybind11::arg("duration"),
    pybind11::arg("message_count"))
  .def_readwrite("path", &rosbag2_storage::FileInformation::path)
  .def_readwrite("starting_time", &rosbag2_storage::FileInformation::starting_time)
  .def_readwrite("duration", &rosbag2_storage::FileInformation::duration)
  .def_readwrite("message_count", &rosbag2_storage::FileInformation::message_count);

  pybind11::class_<rosbag2_storage::BagMetadata>(m, "BagMetadata")
  .def(
    pybind11::init<
      int,
      uint64_t,
      std::string,
      std::vector<std::string>,
      std::vector<rosbag2_storage::FileInformation>,
      std::chrono::nanoseconds,
      std::chrono::time_point<std::chrono::high_resolution_clock>,
      uint64_t,
      std::vector<rosbag2_storage::TopicInformation>,
      std::string,
      std::string>(),
    pybind11::arg("version"),
    pybind11::arg("bag_size"),
    pybind11::arg("storage_identifier"),
    pybind11::arg("relative_file_paths"),
    pybind11::arg("files"),
    pybind11::arg("duration"),
    pybind11::arg("starting_time"),
    pybind11::arg("message_count"),
    pybind11::arg("topics_with_message_count"),
    pybind11::arg("compression_format"),
    pybind11::arg("compression_mode"))
  .def_readwrite("version", &rosbag2_storage::BagMetadata::version)
  .def_readwrite("bag_size", &rosbag2_storage::BagMetadata::bag_size)
  .def_readwrite("storage_identifier", &rosbag2_storage::BagMetadata::storage_identifier)
  .def_readwrite("relative_file_paths", &rosbag2_storage::BagMetadata::relative_file_paths)
  .def_readwrite("files", &rosbag2_storage::BagMetadata::files)
  .def_readwrite("duration", &rosbag2_storage::BagMetadata::duration)
  .def_readwrite("starting_time", &rosbag2_storage::BagMetadata::starting_time)
  .def_readwrite("message_count", &rosbag2_storage::BagMetadata::message_count)
  .def_readwrite(
    "topics_with_message_count",
    &rosbag2_storage::BagMetadata::topics_with_message_count)
  .def_readwrite("compression_format", &rosbag2_storage::BagMetadata::compression_format)
  .def_readwrite("compression_mode", &rosbag2_storage::BagMetadata::compression_mode)
  .def(
    "__repr__", [](const rosbag2_storage::BagMetadata & metadata) {
      return format_bag_meta_data(metadata);
    });
}
