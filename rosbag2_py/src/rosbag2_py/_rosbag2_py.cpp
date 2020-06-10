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
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

namespace rosbag2_py {

class Reader {
public:
  Reader(const std::string &reader_class) {
    if (!reader_class.compare("SequentialReader")) {
      auto sequential_reader =
          std::make_unique<rosbag2_cpp::readers::SequentialReader>();
      reader_ =
          std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
    } else if (!reader_class.compare("SequentialCompressionReader")) {
      auto sequential_reader =
          std::make_unique<rosbag2_compression::SequentialCompressionReader>();
      reader_ =
          std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
    } else {
      throw std::runtime_error{"Reader class type " + reader_class +
                               " not supported."};
    }
  }

  void open(rosbag2_cpp::StorageOptions &storage_options,
            rosbag2_cpp::ConverterOptions &converter_options) {
    reader_->open(storage_options, converter_options);
  }

  bool has_next() { return reader_->has_next(); }

  /// Return a tuple containing the topic name, the serialized ROS message, and
  /// the timestamp.
  pybind11::tuple read_next() {
    const auto next = reader_->read_next();
    rcutils_uint8_array_t rcutils_data = *next->serialized_data.get();
    std::string serialized_data(
        rcutils_data.buffer, rcutils_data.buffer + rcutils_data.buffer_length);
    return pybind11::make_tuple(
        next->topic_name, pybind11::bytes(serialized_data), next->time_stamp);
  }

  /// Return a mapping from topic name to topic type.
  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() {
    return reader_->get_all_topics_and_types();
  }

private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

class Writer {
public:
  Writer(const std::string &writer_class) {
    if (!writer_class.compare("SequentialWriter")) {
      auto sequential_writer =
          std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
      writer_ =
          std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));
    } else if (!writer_class.compare("SequentialCompressionWriter")) {
      auto sequential_writer =
          std::make_unique<rosbag2_compression::SequentialCompressionWriter>();
      writer_ =
          std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));
    } else {
      throw std::runtime_error{"Writer class type " + writer_class +
                               " not supported."};
    }
  }

  void open(rosbag2_cpp::StorageOptions &storage_options,
            rosbag2_cpp::ConverterOptions &converter_options) {
    writer_->open(storage_options, converter_options);
  }

  void create_topic(const rosbag2_storage::TopicMetadata &topic_with_type) {
    writer_->create_topic(topic_with_type);
  }

  void remove_topic(const rosbag2_storage::TopicMetadata &topic_with_type) {
    writer_->remove_topic(topic_with_type);
  }

  /// Write a serialized message to a bag file
  void write(std::string &topic_name, std::string &message, long &time_stamp) {
    auto bag_message =
        std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->topic_name = topic_name;
    bag_message->serialized_data =
        rosbag2_storage::make_serialized_message(message.c_str(), message.length());
    bag_message->time_stamp = (rcutils_time_point_value_t)time_stamp;

    writer_->write(bag_message);
  }

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};
} // namespace rosbag2_py

PYBIND11_MODULE(_rosbag2_py, m) {
  m.doc() = "Python wrapper of the rosbag2_cpp API";

  pybind11::class_<rosbag2_py::Reader>(m, "Reader")
      .def(pybind11::init<const std::string &>())
      .def("open", &rosbag2_py::Reader::open)
      .def("read_next", &rosbag2_py::Reader::read_next)
      .def("has_next", &rosbag2_py::Reader::has_next)
      .def("get_all_topics_and_types",
           &rosbag2_py::Reader::get_all_topics_and_types);

  pybind11::class_<rosbag2_py::Writer>(m, "Writer")
      .def(pybind11::init<const std::string &>())
      .def("open", &rosbag2_py::Writer::open)
      .def("write", &rosbag2_py::Writer::write)
      .def("remove_topic", &rosbag2_py::Writer::remove_topic)
      .def("create_topic", &rosbag2_py::Writer::create_topic);

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
      .def_readwrite(
          "output_serialization_format",
          &rosbag2_cpp::ConverterOptions::output_serialization_format);

  pybind11::class_<rosbag2_storage::TopicMetadata>(m, "TopicMetadata")
      .def(pybind11::init())
      .def_readwrite("name", &rosbag2_storage::TopicMetadata::name)
      .def_readwrite("type", &rosbag2_storage::TopicMetadata::type)
      .def_readwrite("serialization_format",
                     &rosbag2_storage::TopicMetadata::serialization_format)
      .def("equals", &rosbag2_storage::TopicMetadata::operator==);
}
