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

namespace
{

using pybind11::literals::operator""_a;

pybind11::object to_rclpy_duration(std::chrono::nanoseconds duration)
{
  pybind11::object Duration = pybind11::module::import("rclpy.duration").attr("Duration");
  return Duration("nanoseconds"_a = duration.count());
}

std::chrono::nanoseconds from_rclpy_duration(const pybind11::object & duration)
{
  pybind11::int_ nanos = duration.attr("nanoseconds");
  return std::chrono::nanoseconds(nanos);
}

template<typename T>
pybind11::object to_rclpy_time(T time)
{
  pybind11::object Time = pybind11::module::import("rclpy.time").attr("Time");
  return Time(
    "nanoseconds"_a = std::chrono::duration_cast<std::chrono::nanoseconds>(
      time.time_since_epoch()).count());
}

std::chrono::time_point<std::chrono::high_resolution_clock> from_rclpy_time(
  const pybind11::object & time)
{
  pybind11::int_ nanos = time.attr("nanoseconds");
  return std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds(nanos));
}

}  // namespace

PYBIND11_MODULE(_storage, m) {
  m.doc() = "Python wrapper of the rosbag2 utilities API";

  pybind11::class_<rosbag2_cpp::ConverterOptions>(m, "ConverterOptions")
  .def(
    pybind11::init<std::string, std::string>(),
    pybind11::arg("input_serialization_format") = "",
    pybind11::arg("output_serialization_format") = "")
  .def_readwrite(
    "input_serialization_format",
    &rosbag2_cpp::ConverterOptions::input_serialization_format)
  .def_readwrite(
    "output_serialization_format",
    &rosbag2_cpp::ConverterOptions::output_serialization_format);

  using KEY_VALUE_MAP = std::unordered_map<std::string, std::string>;
  pybind11::class_<rosbag2_storage::StorageOptions>(m, "StorageOptions")
  .def(
    pybind11::init<
      std::string, std::string, uint64_t, uint64_t, uint64_t, std::string, std::string, bool,
      KEY_VALUE_MAP>(),
    pybind11::arg("uri"),
    pybind11::arg("storage_id") = "",
    pybind11::arg("max_bagfile_size") = 0,
    pybind11::arg("max_bagfile_duration") = 0,
    pybind11::arg("max_cache_size") = 0,
    pybind11::arg("storage_preset_profile") = "",
    pybind11::arg("storage_config_uri") = "",
    pybind11::arg("snapshot_mode") = false,
    pybind11::arg("custom_data") = KEY_VALUE_MAP{})
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
    &rosbag2_storage::StorageOptions::storage_config_uri)
  .def_readwrite(
    "snapshot_mode",
    &rosbag2_storage::StorageOptions::snapshot_mode)
  .def_readwrite(
    "custom_data",
    &rosbag2_storage::StorageOptions::custom_data);

  pybind11::class_<rosbag2_storage::StorageFilter>(m, "StorageFilter")
  .def(
    pybind11::init<std::vector<std::string>>(),
    pybind11::arg("topics") = std::vector<std::string>())
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
    pybind11::init(
      [](
        std::string path,
        pybind11::object starting_time,
        pybind11::object duration,
        size_t message_count)
      {
        return rosbag2_storage::FileInformation{
          path,
          from_rclpy_time(starting_time),
          from_rclpy_duration(duration),
          message_count
        };
      }),
    pybind11::arg("path"),
    pybind11::arg("starting_time"),
    pybind11::arg("duration"),
    pybind11::arg("message_count")
  )
  .def_readwrite("path", &rosbag2_storage::FileInformation::path)
  .def_property(
    "starting_time",
    [](const rosbag2_storage::FileInformation & self) {
      return to_rclpy_time(self.starting_time);
    },
    [](rosbag2_storage::FileInformation & self, const pybind11::object & value) {
      self.starting_time = from_rclpy_time(value);
    })
  .def_property(
    "duration",
    [](const rosbag2_storage::FileInformation & self) {
      return to_rclpy_duration(self.duration);
    },
    [](rosbag2_storage::FileInformation & self, const pybind11::object & value) {
      self.duration = from_rclpy_duration(value);
    })
  .def_readwrite("duration", &rosbag2_storage::FileInformation::duration)
  .def_readwrite("message_count", &rosbag2_storage::FileInformation::message_count);

  pybind11::class_<rosbag2_storage::BagMetadata>(m, "BagMetadata")
  .def(
    pybind11::init(
      [](
        int version,
        uint64_t bag_size,
        std::string storage_identifier,
        std::vector<std::string> relative_file_paths,
        std::vector<rosbag2_storage::FileInformation> files,
        pybind11::object duration,
        pybind11::object starting_time,
        uint64_t message_count,
        std::vector<rosbag2_storage::TopicInformation> topics_with_message_count,
        std::string compression_format,
        std::string compression_mode,
        std::unordered_map<std::string, std::string> custom_data)
      {
        return rosbag2_storage::BagMetadata{
          version,
          bag_size,
          storage_identifier,
          relative_file_paths,
          files,
          from_rclpy_duration(duration),
          from_rclpy_time(starting_time),
          message_count,
          topics_with_message_count,
          compression_format,
          compression_mode,
          custom_data
        };
      }),
    pybind11::arg("version") = 6,
    pybind11::arg("bag_size") = 0,
    pybind11::arg("storage_identifier") = "",
    pybind11::arg("relative_file_paths") = std::vector<std::string>(),
    pybind11::arg("files") = std::vector<rosbag2_storage::FileInformation>(),
    pybind11::arg("duration") = to_rclpy_duration(std::chrono::nanoseconds{0}),
    pybind11::arg("starting_time") = to_rclpy_time(
      std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds{0})),
    pybind11::arg("message_count") = 0,
    pybind11::arg("topics_with_message_count") = std::vector<rosbag2_storage::TopicInformation>(),
    pybind11::arg("compression_format") = "",
    pybind11::arg("compression_mode") = "",
    pybind11::arg("custom_data") = std::unordered_map<std::string, std::string>())
  .def_readwrite("version", &rosbag2_storage::BagMetadata::version)
  .def_readwrite("bag_size", &rosbag2_storage::BagMetadata::bag_size)
  .def_readwrite("storage_identifier", &rosbag2_storage::BagMetadata::storage_identifier)
  .def_readwrite("relative_file_paths", &rosbag2_storage::BagMetadata::relative_file_paths)
  .def_readwrite("files", &rosbag2_storage::BagMetadata::files)
  .def_property(
    "duration",
    [](const rosbag2_storage::BagMetadata & self) {
      return to_rclpy_duration(self.duration);
    },
    [](rosbag2_storage::BagMetadata & self, const pybind11::object & value) {
      self.duration = from_rclpy_duration(value);
    })
  .def_property(
    "starting_time",
    [](const rosbag2_storage::BagMetadata & self) {
      return to_rclpy_time(self.starting_time);
    },
    [](rosbag2_storage::BagMetadata & self, const pybind11::object & value) {
      self.starting_time = from_rclpy_time(value);
    })
  .def_readwrite("message_count", &rosbag2_storage::BagMetadata::message_count)
  .def_readwrite(
    "topics_with_message_count",
    &rosbag2_storage::BagMetadata::topics_with_message_count)
  .def_readwrite("compression_format", &rosbag2_storage::BagMetadata::compression_format)
  .def_readwrite("compression_mode", &rosbag2_storage::BagMetadata::compression_mode)
  .def_readwrite("custom_data", &rosbag2_storage::BagMetadata::custom_data)
  .def(
    "__repr__", [](const rosbag2_storage::BagMetadata & metadata) {
      return format_bag_meta_data(metadata);
    });
}
