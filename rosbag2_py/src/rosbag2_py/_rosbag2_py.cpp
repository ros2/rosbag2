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

class StorageOptions
{
public:
  StorageOptions() {}

  std::string get_uri()
  {
    return storage_options_.uri;
  }

  void set_uri(const std::string & uri)
  {
    storage_options_.uri = uri;
  }

  std::string get_storage_id()
  {
    return storage_options_.storage_id;
  }

  void set_storage_id(const std::string & id)
  {
    storage_options_.storage_id = id;
  }

  uint64_t get_max_bagfile_size()
  {
    return storage_options_.max_bagfile_size;
  }

  void set_max_bagfile_size(uint64_t size)
  {
    storage_options_.max_bagfile_size = size;
  }

private:
  rosbag2::StorageOptions storage_options_;
};

class ConverterOptions
{
public:
  ConverterOptions() {}

  std::string get_input_serialization_format()
  {
    return converter_options_.input_serialization_format;
  }

  void set_input_serialization_format(const std::string & format)
  {
    converter_options_.input_serialization_format = format;
  }

  std::string get_output_serialization_format()
  {
    return converter_options_.output_serialization_format;
  }

  void set_output_serialization_format(const std::string & format)
  {
    converter_options_.output_serialization_format = format;
  }

private:
  rosbag2::ConverterOptions converter_options_;
};

class SequentialReader
{
public:
  SequentialReader()
  : reader_()
  {}

  void open(
    rosbag2_py::StorageOptions & s_opts_py,
    rosbag2_py::ConverterOptions & c_opts_py)
  {
    rosbag2::StorageOptions storage_options{};
    storage_options.uri = s_opts_py.get_uri();
    storage_options.storage_id = s_opts_py.get_storage_id();

    rosbag2::ConverterOptions converter_options{};
    converter_options.input_serialization_format =
      c_opts_py.get_input_serialization_format();
    converter_options.output_serialization_format =
      c_opts_py.get_output_serialization_format();

    reader_.open(storage_options, converter_options);
  }

  /// Return a pair containing the topic name and the serialized ROS message.
  std::pair<std::string, py::bytes> read_next()
  {
    const auto next = reader_.read_next();
    rcutils_uint8_array_t rcutils_data = *next->serialized_data.get();
    std::string serialized_data(rcutils_data.buffer,
      rcutils_data.buffer + rcutils_data.buffer_length);
    // TODO(jacobperron): Also consider returning the timestamp, next->time_stamp
    return std::make_pair(next->topic_name, py::bytes(serialized_data));
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

  pybind11::class_<rosbag2_py::StorageOptions>(m, "StorageOptions")
  .def(pybind11::init())
  .def("get_uri", &rosbag2_py::StorageOptions::get_uri)
  .def("set_uri", &rosbag2_py::StorageOptions::set_uri)
  .def("get_storage_id", &rosbag2_py::StorageOptions::get_storage_id)
  .def("set_storage_id", &rosbag2_py::StorageOptions::set_storage_id)
  .def("get_max_bagfile_size", &rosbag2_py::StorageOptions::get_max_bagfile_size)
  .def("set_max_bagfile_size", &rosbag2_py::StorageOptions::set_max_bagfile_size);

  pybind11::class_<rosbag2_py::ConverterOptions>(m, "ConverterOptions")
  .def(pybind11::init())
  .def("get_input_serialization_format",
    &rosbag2_py::ConverterOptions::get_input_serialization_format)
  .def("set_input_serialization_format",
    &rosbag2_py::ConverterOptions::set_input_serialization_format)
  .def("get_output_serialization_format",
    &rosbag2_py::ConverterOptions::get_input_serialization_format)
  .def("set_output_serialization_format",
    &rosbag2_py::ConverterOptions::set_output_serialization_format);
}
