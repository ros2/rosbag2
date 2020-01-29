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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <chrono>
#include <string>
#include <utility>
#include <map>

#include "rosbag2/readers/sequential_reader.hpp"
#include "rosbag2/storage_options.hpp"
#include "rosbag2/converter_options.hpp"
// #include "rmw/rmw.h"

namespace rosbag2_py
{

class SequentialReader
{
public:
  SequentialReader()
  : reader_()
  {}

  void open(
    rosbag2::StorageOptions & storage_options,
    rosbag2::ConverterOptions & converter_options)
  {
    reader_.open(storage_options, converter_options);
  }

  /// Return a pair containing the topic name and the serialized ROS message.
  pybind11::tuple read_next()
  {
    const auto next = reader_.read_next();
    rcutils_uint8_array_t rcutils_data = *next->serialized_data.get();
    std::string serialized_data(rcutils_data.buffer,
      rcutils_data.buffer + rcutils_data.buffer_length);
    return pybind11::make_tuple(next->topic_name, py::bytes(serialized_data), next->time_stamp);
  }

  bool has_next()
  {
    return reader_.has_next();
  }

  /// Return a mapping from topic name to topic type.
  std::map<std::string, std::string> get_all_topics_and_types()
  {
    // TODO(jacobperron): pybind TopicMetadata and return the vector of it directly
    auto topics = reader_.get_all_topics_and_types();
    // We're making an assumption that there is only one type per topic
    std::map<std::string, std::string> output;
    for (const rosbag2::TopicMetadata & topic_metadata : topics) {
      output[topic_metadata.name] = topic_metadata.type;
    }
    return output;
  }

private:
  rosbag2::readers::SequentialReader reader_;
};

}  // namespace rosbag2_py

PYBIND11_MODULE(_rosbag2_py, m) {
  m.doc() = "Python wrapper of rosbag2 implementation";

  py::class_<rosbag2_py::SequentialReader>(m, "SequentialReader")
  .def(py::init())
  .def("open", &rosbag2_py::SequentialReader::open)
  .def("read_next", &rosbag2_py::SequentialReader::read_next)
  .def("has_next", &rosbag2_py::SequentialReader::has_next)
  .def("get_all_topics_and_types", &rosbag2_py::SequentialReader::get_all_topics_and_types);

  pybind11::class_<rosbag2::StorageOptions>(m, "StorageOptions")
  .def(pybind11::init())
  .def_readwrite("uri", &rosbag2::StorageOptions::uri)
  .def_readwrite("storage_id", &rosbag2::StorageOptions::storage_id)
  .def_readwrite("max_bagfile_size",
    &rosbag2::StorageOptions::max_bagfile_size);

  pybind11::class_<rosbag2::ConverterOptions>(m, "ConverterOptions")
  .def(pybind11::init())
  .def_readwrite("input_serialization_format",
    &rosbag2::ConverterOptions::input_serialization_format)
  .def_readwrite("output_serialization_format",
    &rosbag2::ConverterOptions::output_serialization_format);
}
