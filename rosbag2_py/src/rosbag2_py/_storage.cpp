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
#include "rosbag2_storage/default_storage_id.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_interfaces/base_read_interface.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "format_bag_metadata.hpp"
#include "pybind11.hpp"

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
      int64_t, int64_t, KEY_VALUE_MAP>(),
    pybind11::arg("uri"),
    pybind11::arg("storage_id") = "",
    pybind11::arg("max_bagfile_size") = 0,
    pybind11::arg("max_bagfile_duration") = 0,
    pybind11::arg("max_cache_size") = 0,
    pybind11::arg("storage_preset_profile") = "",
    pybind11::arg("storage_config_uri") = "",
    pybind11::arg("snapshot_mode") = false,
    pybind11::arg("start_time_ns") = -1,
    pybind11::arg("end_time_ns") = -1,
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
    "start_time_ns",
    &rosbag2_storage::StorageOptions::start_time_ns)
  .def_readwrite(
    "end_time_ns",
    &rosbag2_storage::StorageOptions::end_time_ns)
  .def_readwrite(
    "custom_data",
    &rosbag2_storage::StorageOptions::custom_data);

  pybind11::class_<rosbag2_storage::StorageFilter>(m, "StorageFilter")
  .def(
    pybind11::init<std::vector<std::string>, std::string, std::string>(),
    pybind11::arg("topics") = std::vector<std::string>(),
    pybind11::arg("topics_regex") = "",
    pybind11::arg("topics_regex_to_exclude") = "")
  .def_readwrite("topics", &rosbag2_storage::StorageFilter::topics)
  .def_readwrite("topics_regex", &rosbag2_storage::StorageFilter::topics_regex)
  .def_readwrite(
    "topics_regex_to_exclude",
    &rosbag2_storage::StorageFilter::topics_regex_to_exclude);

  pybind11::class_<rosbag2_storage::MessageDefinition>(m, "MessageDefinition")
  .def(
    pybind11::init<std::string, std::string, std::string, std::string>(),
    pybind11::arg("topic_type"),
    pybind11::arg("encoding"),
    pybind11::arg("encoded_message_definition"),
    pybind11::arg("type_hash"))
  .def_readwrite("topic_type", &rosbag2_storage::MessageDefinition::topic_type)
  .def_readwrite("encoding", &rosbag2_storage::MessageDefinition::encoding)
  .def_readwrite(
    "encoded_message_definition",
    &rosbag2_storage::MessageDefinition::encoded_message_definition)
  .def_readwrite("type_hash", &rosbag2_storage::MessageDefinition::type_hash);

  pybind11::enum_<rmw_qos_history_policy_t>(m, "rmw_qos_history_policy_t")
  .value("RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT", RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT)
  .value("RMW_QOS_POLICY_HISTORY_KEEP_LAST", RMW_QOS_POLICY_HISTORY_KEEP_LAST)
  .value("RMW_QOS_POLICY_HISTORY_KEEP_ALL", RMW_QOS_POLICY_HISTORY_KEEP_ALL)
  .value("RMW_QOS_POLICY_HISTORY_UNKNOWN", RMW_QOS_POLICY_HISTORY_UNKNOWN);

  pybind11::enum_<rmw_qos_reliability_policy_t>(m, "rmw_qos_reliability_policy_t")
  .value("RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT)
  .value("RMW_QOS_POLICY_RELIABILITY_RELIABLE", RMW_QOS_POLICY_RELIABILITY_RELIABLE)
  .value("RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
  .value("RMW_QOS_POLICY_RELIABILITY_UNKNOWN", RMW_QOS_POLICY_RELIABILITY_UNKNOWN);

  pybind11::enum_<rmw_qos_durability_policy_t>(m, "rmw_qos_durability_policy_t")
  .value("RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT", RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT)
  .value("RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL", RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
  .value("RMW_QOS_POLICY_DURABILITY_VOLATILE", RMW_QOS_POLICY_DURABILITY_VOLATILE)
  .value("RMW_QOS_POLICY_DURABILITY_UNKNOWN", RMW_QOS_POLICY_DURABILITY_UNKNOWN);


  pybind11::enum_<rmw_qos_liveliness_policy_t>(m, "rmw_qos_liveliness_policy_t")
  .value("RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT", RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT)
  .value("RMW_QOS_POLICY_LIVELINESS_AUTOMATIC", RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
  .value("RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC", RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
  .value("RMW_QOS_POLICY_LIVELINESS_UNKNOWN", RMW_QOS_POLICY_LIVELINESS_UNKNOWN);

  pybind11::class_<rclcpp::Duration>(m, "Duration")
  .def(
    pybind11::init<int32_t, uint32_t>(),
    pybind11::arg("seconds"),
    pybind11::arg("nanoseconds"));

  pybind11::class_<rclcpp::QoS>(m, "QoS")
  .def(
    pybind11::init<size_t>(),
    pybind11::arg("history_depth"))
  .def("keep_last", &rclcpp::QoS::keep_last)
  .def("keep_all", &rclcpp::QoS::keep_all)
  .def("reliable", &rclcpp::QoS::reliable)
  .def("best_effort", &rclcpp::QoS::best_effort)
  .def("durability_volatile", &rclcpp::QoS::durability_volatile)
  .def("transient_local", &rclcpp::QoS::transient_local)
  .def("history", pybind11::overload_cast<rmw_qos_history_policy_t>(&rclcpp::QoS::history))
  .def(
    "reliability",
    pybind11::overload_cast<rmw_qos_reliability_policy_t>(&rclcpp::QoS::reliability))
  .def("durability", pybind11::overload_cast<rmw_qos_durability_policy_t>(&rclcpp::QoS::durability))
  .def("liveliness", pybind11::overload_cast<rmw_qos_liveliness_policy_t>(&rclcpp::QoS::liveliness))
  .def("deadline", pybind11::overload_cast<const rclcpp::Duration &>(&rclcpp::QoS::deadline))
  .def("lifespan", pybind11::overload_cast<const rclcpp::Duration &>(&rclcpp::QoS::lifespan))
  .def(
    "liveliness_lease_duration",
    pybind11::overload_cast<const rclcpp::Duration &>(&rclcpp::QoS::liveliness_lease_duration))
  .def(
    "avoid_ros_namespace_conventions",
    pybind11::overload_cast<bool>(&rclcpp::QoS::avoid_ros_namespace_conventions));

  pybind11::class_<rosbag2_storage::TopicMetadata>(m, "TopicMetadata")
  .def(
    pybind11::init<std::string, std::string, std::string, std::vector<rclcpp::QoS>, std::string>(),
    pybind11::arg("name"),
    pybind11::arg("type"),
    pybind11::arg("serialization_format"),
    pybind11::arg("offered_qos_profiles") = std::vector<rclcpp::QoS>(),
    pybind11::arg("type_description_hash") = "")
  .def_readwrite("name", &rosbag2_storage::TopicMetadata::name)
  .def_readwrite("type", &rosbag2_storage::TopicMetadata::type)
  .def_readwrite(
    "serialization_format",
    &rosbag2_storage::TopicMetadata::serialization_format)
  .def_readwrite(
    "offered_qos_profiles",
    &rosbag2_storage::TopicMetadata::offered_qos_profiles)
  .def_readwrite(
    "type_description_hash",
    &rosbag2_storage::TopicMetadata::type_description_hash)
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
        std::unordered_map<std::string, std::string> custom_data,
        std::string ros_distro)
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
          custom_data,
          ros_distro,
        };
      }),
    pybind11::arg("version") = rosbag2_storage::BagMetadata{}.version,
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
    pybind11::arg("custom_data") = std::unordered_map<std::string, std::string>(),
    pybind11::arg("ros_distro") = "")
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
  .def_readwrite("ros_distro", &rosbag2_storage::BagMetadata::ros_distro)
  .def(
    "__repr__", [](const rosbag2_storage::BagMetadata & metadata) {
      return rosbag2_py::format_bag_meta_data(metadata);
    });

  pybind11::enum_<rosbag2_storage::ReadOrder::SortBy>(m, "ReadOrderSortBy")
  .value("ReceivedTimestamp", rosbag2_storage::ReadOrder::ReceivedTimestamp)
  .value("PublishedTimestamp", rosbag2_storage::ReadOrder::PublishedTimestamp)
  .value("File", rosbag2_storage::ReadOrder::File);

  pybind11::class_<rosbag2_storage::ReadOrder>(m, "ReadOrder")
  .def(
    pybind11::init<rosbag2_storage::ReadOrder::SortBy, bool>(),
    pybind11::arg("sort_by") = rosbag2_storage::ReadOrder{}.sort_by,
    pybind11::arg("reverse") = rosbag2_storage::ReadOrder{}.reverse)
  .def_readwrite("sort_by", &rosbag2_storage::ReadOrder::sort_by)
  .def_readwrite("reverse", &rosbag2_storage::ReadOrder::reverse);

  m.def(
    "get_default_storage_id",
    &rosbag2_storage::get_default_storage_id,
    "Returns the default storage ID used when unspecified in StorageOptions");

  pybind11::class_<rosbag2_storage::MetadataIo>(m, "MetadataIo")
  .def(pybind11::init<>())
  .def("write_metadata", &rosbag2_storage::MetadataIo::write_metadata)
  .def("read_metadata", &rosbag2_storage::MetadataIo::read_metadata)
  .def("metadata_file_exists", &rosbag2_storage::MetadataIo::metadata_file_exists)
  .def("serialize_metadata", &rosbag2_storage::MetadataIo::serialize_metadata)
  .def("deserialize_metadata", &rosbag2_storage::MetadataIo::deserialize_metadata)
  ;
}
