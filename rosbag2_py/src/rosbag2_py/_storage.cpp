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

// Ignore -Wunused-value for clang.
// Based on https://github.com/pybind/pybind11/issues/2225
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-value"
#endif
#include <pybind11/pybind11.h>
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
#include <pybind11/stl.h>

#include <string>
#include <vector>

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

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

  pybind11::class_<rosbag2_cpp::StorageOptions>(m, "StorageOptions")
  .def(
    pybind11::init<std::string, std::string, uint64_t, uint64_t, uint64_t>(),
    pybind11::arg("uri"),
    pybind11::arg("storage_id"),
    pybind11::arg("max_bagfile_size") = 0,
    pybind11::arg("max_bagfile_duration") = 0,
    pybind11::arg("max_cache_size") = 0)
  .def_readwrite("uri", &rosbag2_cpp::StorageOptions::uri)
  .def_readwrite("storage_id", &rosbag2_cpp::StorageOptions::storage_id)
  .def_readwrite(
    "max_bagfile_size",
    &rosbag2_cpp::StorageOptions::max_bagfile_size)
  .def_readwrite(
    "max_bagfile_duration",
    &rosbag2_cpp::StorageOptions::max_bagfile_duration)
  .def_readwrite(
    "max_cache_size",
    &rosbag2_cpp::StorageOptions::max_cache_size);

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
}
