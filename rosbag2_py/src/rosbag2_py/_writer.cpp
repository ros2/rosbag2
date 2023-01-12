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

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>

#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/plugins/plugin_utils.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_cpp/serialization_format_converter_factory.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "./pybind11.hpp"

namespace rosbag2_py
{

template<typename T>
class Writer : public rosbag2_cpp::Writer
{
public:
  Writer()
  : rosbag2_cpp::Writer(std::make_unique<T>())
  {}

  /// Write a serialized message to a bag file
  void write(
    const std::string & topic_name, const std::string & message,
    const rcutils_time_point_value_t & time_stamp)
  {
    auto bag_message =
      std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->topic_name = topic_name;
    bag_message->serialized_data =
      rosbag2_storage::make_serialized_message(message.c_str(), message.length());
    bag_message->time_stamp = time_stamp;

    rosbag2_cpp::Writer::write(bag_message);
  }
};

std::unordered_set<std::string> get_registered_writers()
{
  return rosbag2_cpp::plugins::get_class_plugins<
    rosbag2_storage::storage_interfaces::ReadWriteInterface>();
}

std::string get_package_for_registered_writer(const std::string & class_name)
{
  return rosbag2_cpp::plugins::package_for_class_plugin<
    rosbag2_storage::storage_interfaces::ReadWriteInterface>(class_name);
}

std::unordered_set<std::string> get_registered_compressors()
{
  return rosbag2_cpp::plugins::get_class_plugins<
    rosbag2_compression::BaseCompressorInterface>();
}

std::unordered_set<std::string> get_registered_serializers()
{
  auto serializers = rosbag2_cpp::plugins::get_class_plugins<
    rosbag2_cpp::converter_interfaces::SerializationFormatSerializer>();
  auto converters = rosbag2_cpp::plugins::get_class_plugins<
    rosbag2_cpp::converter_interfaces::SerializationFormatConverter>();
  serializers.insert(converters.begin(), converters.end());
  return serializers;
}

}  // namespace rosbag2_py

using PyWriter = rosbag2_py::Writer<rosbag2_cpp::writers::SequentialWriter>;
using PyCompressionWriter = rosbag2_py::Writer<rosbag2_compression::SequentialCompressionWriter>;

PYBIND11_MODULE(_writer, m) {
  m.doc() = "Python wrapper of the rosbag2_cpp writer API";

  pybind11::class_<PyWriter>(m, "SequentialWriter")
  .def(pybind11::init())
  .def(
    "open",
    pybind11::overload_cast<
      const rosbag2_storage::StorageOptions &, const rosbag2_cpp::ConverterOptions &
    >(&PyWriter::open))
  .def("write", &PyWriter::write)
  .def("remove_topic", &PyWriter::remove_topic)
  .def("create_topic", &PyWriter::create_topic)
  .def("take_snapshot", &PyWriter::take_snapshot)
  .def("split_bagfile", &PyWriter::split_bagfile)
  ;

  pybind11::class_<PyCompressionWriter>(m, "SequentialCompressionWriter")
  .def(pybind11::init())
  .def(
    "open",
    pybind11::overload_cast<
      const rosbag2_storage::StorageOptions &, const rosbag2_cpp::ConverterOptions &
    >(&PyCompressionWriter::open))
  .def("write", &PyCompressionWriter::write)
  .def("remove_topic", &PyCompressionWriter::remove_topic)
  .def("create_topic", &PyCompressionWriter::create_topic)
  .def("take_snapshot", &PyCompressionWriter::take_snapshot)
  .def("split_bagfile", &PyCompressionWriter::split_bagfile)
  ;

  m.def(
    "get_registered_writers",
    &rosbag2_py::get_registered_writers,
    "Returns list of discovered plugins that support rosbag2 recording");
  m.def(
    "get_package_for_registered_writer",
    &rosbag2_py::get_package_for_registered_writer,
    "Returns the name of the package that provides a given writer plugin.");

  m.def(
    "get_registered_compressors",
    &rosbag2_py::get_registered_compressors,
    "Returns list of compression plugins available for rosbag2 recording");

  m.def(
    "get_registered_serializers",
    &rosbag2_py::get_registered_serializers,
    "Returns list of serialization format plugins available for rosbag2 recording");
}
