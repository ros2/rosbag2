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

#include "rosbag2_compression/compression_factory.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_compression/compression_traits.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/converter_traits.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_cpp/serialization_format_converter_factory.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/storage_traits.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "./py_utils.hpp"
#include "./pybind11.hpp"

namespace rosbag2_py
{

template<typename T>
class Writer
{
public:
  Writer()
  : writer_(std::make_unique<rosbag2_cpp::Writer>(std::make_unique<T>()))
  {
  }

  void open(
    rosbag2_storage::StorageOptions & storage_options,
    rosbag2_cpp::ConverterOptions & converter_options)
  {
    writer_->open(storage_options, converter_options);
  }

  void create_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
  {
    writer_->create_topic(topic_with_type);
  }

  void remove_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
  {
    writer_->remove_topic(topic_with_type);
  }

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

    writer_->write(bag_message);
  }

protected:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

std::unordered_set<std::string> get_registered_writers()
{
  return get_class_plugins<rosbag2_storage::storage_interfaces::ReadWriteInterface>();
}

std::unordered_set<std::string> get_registered_compressors()
{
  return get_class_plugins<rosbag2_compression::BaseCompressorInterface>();
}

std::unordered_set<std::string> get_registered_serializers()
{
  return get_class_plugins<rosbag2_cpp::converter_interfaces::SerializationFormatSerializer>();
}

}  // namespace rosbag2_py

PYBIND11_MODULE(_writer, m) {
  m.doc() = "Python wrapper of the rosbag2_cpp writer API";

  pybind11::class_<rosbag2_py::Writer<rosbag2_cpp::writers::SequentialWriter>>(
    m, "SequentialWriter")
  .def(pybind11::init())
  .def("open", &rosbag2_py::Writer<rosbag2_cpp::writers::SequentialWriter>::open)
  .def("write", &rosbag2_py::Writer<rosbag2_cpp::writers::SequentialWriter>::write)
  .def("remove_topic", &rosbag2_py::Writer<rosbag2_cpp::writers::SequentialWriter>::remove_topic)
  .def("create_topic", &rosbag2_py::Writer<rosbag2_cpp::writers::SequentialWriter>::create_topic);

  pybind11::class_<rosbag2_py::Writer<rosbag2_compression::SequentialCompressionWriter>>(
    m, "SequentialCompressionWriter")
  .def(pybind11::init())
  .def("open", &rosbag2_py::Writer<rosbag2_compression::SequentialCompressionWriter>::open)
  .def("write", &rosbag2_py::Writer<rosbag2_compression::SequentialCompressionWriter>::write)
  .def(
    "remove_topic",
    &rosbag2_py::Writer<rosbag2_compression::SequentialCompressionWriter>::remove_topic)
  .def(
    "create_topic",
    &rosbag2_py::Writer<rosbag2_compression::SequentialCompressionWriter>::create_topic);

  m.def(
    "get_registered_writers",
    &rosbag2_py::get_registered_writers,
    "Returns list of discovered plugins that support rosbag2 recording");

  m.def(
    "get_registered_compressors",
    &rosbag2_py::get_registered_compressors,
    "Returns list of compression plugins available for rosbag2 recording");

  m.def(
    "get_registered_serializers",
    &rosbag2_py::get_registered_serializers,
    "Returns list of serialization format plugins available for rosbag2 recording");
}
