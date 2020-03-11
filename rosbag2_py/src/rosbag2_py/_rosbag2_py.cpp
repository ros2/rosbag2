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

#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

namespace rosbag2_py
{

class SequentialReader
{
public:
  SequentialReader()
  : reader_()
  {}

  void open(
    rosbag2_cpp::StorageOptions & storage_options,
    rosbag2_cpp::ConverterOptions & converter_options)
  {
    reader_.open(storage_options, converter_options);
  }

  bool has_next()
  {
    return reader_.has_next();
  }

  /// Return a pair containing the topic name and the serialized ROS message.
  pybind11::tuple read_next()
  {
    const auto next = reader_.read_next();
    rcutils_uint8_array_t rcutils_data = *next->serialized_data.get();
    std::string serialized_data(rcutils_data.buffer,
      rcutils_data.buffer + rcutils_data.buffer_length);
    return pybind11::make_tuple(next->topic_name, pybind11::bytes(serialized_data),
             next->time_stamp);
  }

  /// Return a mapping from topic name to topic type.
  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types()
  {
    return reader_.get_all_topics_and_types();
  }

private:
  rosbag2_cpp::readers::SequentialReader reader_;
};

}  // namespace rosbag2_py

PYBIND11_MODULE(_rosbag2_py, m) {
  m.doc() = "Python wrapper of the rosbag2_cpp API";

  pybind11::class_<rosbag2_py::SequentialReader>(m, "SequentialReader")
  .def(pybind11::init())
  .def("open", &rosbag2_py::SequentialReader::open)
  .def("read_next", &rosbag2_py::SequentialReader::read_next)
  .def("has_next", &rosbag2_py::SequentialReader::has_next)
  .def("get_all_topics_and_types", &rosbag2_py::SequentialReader::get_all_topics_and_types);

  pybind11::class_<rosbag2_cpp::StorageOptions>(m, "StorageOptions")
  .def(pybind11::init())
  .def_readwrite("uri", &rosbag2_cpp::StorageOptions::uri)
  .def_readwrite("storage_id", &rosbag2_cpp::StorageOptions::storage_id)
  .def_readwrite("max_bagfile_size",
    &rosbag2_cpp::StorageOptions::max_bagfile_size);

  pybind11::class_<rosbag2_cpp::ConverterOptions>(m, "ConverterOptions")
  .def(pybind11::init())
  .def_readwrite("input_serialization_format",
    &rosbag2_cpp::ConverterOptions::input_serialization_format)
  .def_readwrite("output_serialization_format",
    &rosbag2_cpp::ConverterOptions::output_serialization_format);

  pybind11::class_<rosbag2_storage::TopicMetadata>(m, "TopicMetadata")
  .def(pybind11::init())
  .def_readwrite("name", &rosbag2_storage::TopicMetadata::name)
  .def_readwrite("type", &rosbag2_storage::TopicMetadata::type)
  .def_readwrite("serialization_format",
    &rosbag2_storage::TopicMetadata::serialization_format)
  .def("equals", &rosbag2_storage::TopicMetadata::operator==);
}
