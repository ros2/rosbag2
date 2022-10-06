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
#include <vector>

#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/plugins/plugin_utils.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "./pybind11.hpp"

namespace rosbag2_py
{

template<typename T>
class Reader : public rosbag2_cpp::Reader
{
public:
  Reader()
  : rosbag2_cpp::Reader(std::make_unique<T>())
  {
  }

  /// Return a tuple containing the topic name, the serialized ROS message, and
  /// the timestamp.
  pybind11::tuple read_next()
  {
    const auto next = rosbag2_cpp::Reader::read_next();
    rcutils_uint8_array_t rcutils_data = *next->serialized_data.get();
    std::string serialized_data(rcutils_data.buffer,
      rcutils_data.buffer + rcutils_data.buffer_length);
    return pybind11::make_tuple(
      next->topic_name, pybind11::bytes(serialized_data), next->time_stamp);
  }
};

std::unordered_set<std::string> get_registered_readers()
{
  std::unordered_set<std::string> combined_plugins = rosbag2_cpp::plugins::get_class_plugins
    <rosbag2_storage::storage_interfaces::ReadWriteInterface>();

  std::unordered_set<std::string> read_only_plugins = rosbag2_cpp::plugins::get_class_plugins
    <rosbag2_storage::storage_interfaces::ReadOnlyInterface>();

  // Merge read/write and read-only plugin sets
  combined_plugins.insert(read_only_plugins.begin(), read_only_plugins.end());

  return combined_plugins;
}

}  // namespace rosbag2_py

using PyReader = rosbag2_py::Reader<rosbag2_cpp::readers::SequentialReader>;
using PyCompressionReader = rosbag2_py::Reader<rosbag2_compression::SequentialCompressionReader>;

PYBIND11_MODULE(_reader, m) {
  m.doc() = "Python wrapper of the rosbag2_cpp reader API";

  pybind11::class_<PyReader>(m, "SequentialReader")
  .def(pybind11::init())
  .def("open_uri", pybind11::overload_cast<const std::string &>(&PyReader::open))
  .def(
    "open",
    pybind11::overload_cast<
      const rosbag2_storage::StorageOptions &, const rosbag2_cpp::ConverterOptions &
    >(&PyReader::open))
  .def("set_read_order", &PyReader::set_read_order)
  .def("read_next", &PyReader::read_next)
  .def("has_next", &PyReader::has_next)
  .def("get_metadata", &PyReader::get_metadata)
  .def("get_all_topics_and_types", &PyReader::get_all_topics_and_types)
  .def("set_filter", &PyReader::set_filter)
  .def("reset_filter", &PyReader::reset_filter)
  .def("seek", &PyReader::seek);

  pybind11::class_<PyCompressionReader>(m, "SequentialCompressionReader")
  .def(pybind11::init())
  .def("open_uri", pybind11::overload_cast<const std::string &>(&PyCompressionReader::open))
  .def(
    "open",
    pybind11::overload_cast<
      const rosbag2_storage::StorageOptions &, const rosbag2_cpp::ConverterOptions &
    >(&PyCompressionReader::open))
  .def("set_read_order", &PyCompressionReader::set_read_order)
  .def("read_next", &PyCompressionReader::read_next)
  .def("has_next", &PyCompressionReader::has_next)
  .def("get_metadata", &PyCompressionReader::get_metadata)
  .def("get_all_topics_and_types", &PyCompressionReader::get_all_topics_and_types)
  .def("set_filter", &PyCompressionReader::set_filter)
  .def("reset_filter", &PyCompressionReader::reset_filter)
  .def("seek", &PyCompressionReader::seek);
  m.def(
    "get_registered_readers",
    &rosbag2_py::get_registered_readers,
    "Returns list of discovered plugins that support rosbag2 playback.");
}
