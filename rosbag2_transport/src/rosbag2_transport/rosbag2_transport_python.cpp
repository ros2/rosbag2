// Copyright 2018 Open Source Robotics Foundation, Inc.
// Copyright 2020, TNG Technology Consulting GmbH.
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

#include <Python.h>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"

#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/rosbag2_transport.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rmw/rmw.h"

namespace
{
/// Convert a Python3 unicode string to a native C++ std::string
std::string PyObject_AsStdString(PyObject * object)
{
  PyObject * python_string = nullptr;
  if (PyUnicode_Check(object)) {
    python_string = PyUnicode_AsUTF8String(object);
  } else {
    throw std::runtime_error("Unable to decode Python string to std::string.");
  }
  return std::string(PyBytes_AsString(python_string));
}

/// Get the rmw_qos_profile_t pointer from the rclpy QoSProfile
rmw_qos_profile_t * PyQoSProfile_AsRmwQoSProfile(PyObject * object)
{
  auto py_capsule = PyObject_CallMethod(object, "get_c_qos_profile", "");
  return reinterpret_cast<rmw_qos_profile_t *>(
    PyCapsule_GetPointer(py_capsule, "rmw_qos_profile_t"));
}

std::unordered_map<std::string, rclcpp::QoS> PyObject_AsTopicQoSMap(PyObject * object)
{
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_overrides{};
  if (PyDict_Check(object)) {
    PyObject * key{nullptr};
    PyObject * value{nullptr};
    Py_ssize_t pos{0};
    while (PyDict_Next(object, &pos, &key, &value)) {
      auto topic_name = PyObject_AsStdString(key);
      auto rmw_qos_profile = PyQoSProfile_AsRmwQoSProfile(value);
      auto qos_init = rclcpp::QoSInitialization::from_rmw(*rmw_qos_profile);
      auto qos_profile = rclcpp::QoS(qos_init, *rmw_qos_profile);
      topic_qos_overrides.insert({topic_name, qos_profile});
    }
  } else {
    throw std::runtime_error{"QoS profile overrides object is not a Python dictionary."};
  }
  return topic_qos_overrides;
}

}  // namespace

static PyObject *
rosbag2_transport_record(PyObject * Py_UNUSED(self), PyObject * args, PyObject * kwargs)
{
  rosbag2_storage::StorageOptions storage_options{};
  rosbag2_transport::RecordOptions record_options{};

  static const char * kwlist[] = {
    "uri",
    "storage_id",
    "serialization_format",
    "node_prefix",
    "compression_mode",
    "compression_format",
    "all",
    "no_discovery",
    "polling_interval",
    "max_bagfile_size",
    "max_bagfile_duration",
    "max_cache_size",
    "topics",
    "include_hidden_topics",
    "qos_profile_overrides",
    "resilient_storage_writing",
    "storage_config_file",
    nullptr};

  char * uri = nullptr;
  char * storage_id = nullptr;
  char * serilization_format = nullptr;
  char * node_prefix = nullptr;
  char * compression_mode = nullptr;
  char * compression_format = nullptr;
  PyObject * qos_profile_overrides = nullptr;
  bool all = false;
  bool no_discovery = false;
  uint64_t polling_interval_ms = 100;
  unsigned long long max_bagfile_size = 0;  // NOLINT
  unsigned long long max_bagfile_duration = 0;  // NOLINT
  uint64_t max_cache_size = 0u;
  PyObject * topics = nullptr;
  bool include_hidden_topics = false;
  bool resilient_storage_writing = false;
  char * storage_config_file = nullptr;
  if (
    !PyArg_ParseTupleAndKeywords(
      args, kwargs, "ssssss|bbKKKKObObs", const_cast<char **>(kwlist),
      &uri,
      &storage_id,
      &serilization_format,
      &node_prefix,
      &compression_mode,
      &compression_format,
      &all,
      &no_discovery,
      &polling_interval_ms,
      &max_bagfile_size,
      &max_bagfile_duration,
      &max_cache_size,
      &topics,
      &include_hidden_topics,
      &qos_profile_overrides,
      &resilient_storage_writing,
      &storage_config_file
  ))
  {
    return nullptr;
  }

  storage_options.uri = std::string(uri);
  storage_options.storage_id = std::string(storage_id);
  storage_options.storage_config_uri = std::string(storage_config_file);
  storage_options.max_bagfile_size = (uint64_t) max_bagfile_size;
  storage_options.max_bagfile_duration = static_cast<uint64_t>(max_bagfile_duration);
  storage_options.max_cache_size = max_cache_size;
  storage_options.resilient_storage_writing = resilient_storage_writing;
  record_options.all = all;
  record_options.is_discovery_disabled = no_discovery;
  record_options.topic_polling_interval = std::chrono::milliseconds(polling_interval_ms);
  record_options.node_prefix = std::string(node_prefix);
  record_options.compression_mode = std::string(compression_mode);
  record_options.compression_format = compression_format;
  record_options.include_hidden_topics = include_hidden_topics;

  rosbag2_compression::CompressionOptions compression_options{
    record_options.compression_format,
    rosbag2_compression::compression_mode_from_string(record_options.compression_mode)
  };

  auto topic_qos_overrides = PyObject_AsTopicQoSMap(qos_profile_overrides);
  record_options.topic_qos_profile_overrides = topic_qos_overrides;

  if (topics) {
    PyObject * topic_iterator = PyObject_GetIter(topics);
    if (topic_iterator != nullptr) {
      PyObject * topic;
      while ((topic = PyIter_Next(topic_iterator))) {
        record_options.topics.emplace_back(PyUnicode_AsUTF8(topic));

        Py_DECREF(topic);
      }
      Py_DECREF(topic_iterator);
    }
  }
  record_options.rmw_serialization_format = std::string(serilization_format).empty() ?
    rmw_get_serialization_format() :
    serilization_format;

  // Specify defaults
  auto info = std::make_shared<rosbag2_cpp::Info>();
  auto reader = std::make_shared<rosbag2_cpp::Reader>(
    std::make_unique<rosbag2_cpp::readers::SequentialReader>());
  std::shared_ptr<rosbag2_cpp::Writer> writer;
  // Change writer based on recording options
  if (record_options.compression_format == "zstd") {
    writer = std::make_shared<rosbag2_cpp::Writer>(
      std::make_unique<rosbag2_compression::SequentialCompressionWriter>(compression_options));
  } else {
    writer = std::make_shared<rosbag2_cpp::Writer>(
      std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
  }

  rosbag2_transport::Rosbag2Transport transport(reader, writer, info);
  transport.init();
  transport.record(storage_options, record_options);
  transport.shutdown();

  Py_RETURN_NONE;
}

std::vector<std::string> get_topic_remapping_options(PyObject * topic_remapping)
{
  std::vector<std::string> topic_remapping_options = {"--ros-args"};
  if (topic_remapping) {
    PyObject * topic_remapping_iterator = PyObject_GetIter(topic_remapping);
    if (topic_remapping_iterator) {
      PyObject * topic;
      while ((topic = PyIter_Next(topic_remapping_iterator))) {
        topic_remapping_options.emplace_back("--remap");
        topic_remapping_options.emplace_back(PyUnicode_AsUTF8(topic));

        Py_DECREF(topic);
      }
      Py_DECREF(topic_remapping_iterator);
    }
  }
  return topic_remapping_options;
}

static PyObject *
rosbag2_transport_play(PyObject * Py_UNUSED(self), PyObject * args, PyObject * kwargs)
{
  rosbag2_transport::PlayOptions play_options{};
  rosbag2_storage::StorageOptions storage_options{};

  static const char * kwlist[] = {
    "uri",
    "storage_id",
    "node_prefix",
    "read_ahead_queue_size",
    "rate",
    "topics",
    "qos_profile_overrides",
    "loop",
    "topic_remapping",
    "storage_config_file",
    nullptr
  };

  char * uri;
  char * storage_id;
  char * node_prefix;
  size_t read_ahead_queue_size;
  float rate;
  PyObject * topics = nullptr;
  PyObject * qos_profile_overrides{nullptr};
  bool loop = false;
  PyObject * topic_remapping = nullptr;
  char * storage_config_file = nullptr;
  if (!PyArg_ParseTupleAndKeywords(
      args, kwargs, "sss|kfOObOs", const_cast<char **>(kwlist),
      &uri,
      &storage_id,
      &node_prefix,
      &read_ahead_queue_size,
      &rate,
      &topics,
      &qos_profile_overrides,
      &loop,
      &topic_remapping,
      &storage_config_file))
  {
    return nullptr;
  }

  storage_options.uri = std::string(uri);
  storage_options.storage_id = std::string(storage_id);
  storage_options.storage_config_uri = std::string(storage_config_file);

  play_options.node_prefix = std::string(node_prefix);
  play_options.read_ahead_queue_size = read_ahead_queue_size;
  play_options.rate = rate;
  play_options.loop = loop;

  if (topics) {
    PyObject * topic_iterator = PyObject_GetIter(topics);
    if (topic_iterator != nullptr) {
      PyObject * topic = nullptr;
      while ((topic = PyIter_Next(topic_iterator))) {
        play_options.topics_to_filter.emplace_back(PyUnicode_AsUTF8(topic));

        Py_DECREF(topic);
      }
      Py_DECREF(topic_iterator);
    }
  }

  auto topic_qos_overrides = PyObject_AsTopicQoSMap(qos_profile_overrides);
  play_options.topic_qos_profile_overrides = topic_qos_overrides;

  play_options.topic_remapping_options = get_topic_remapping_options(topic_remapping);

  rosbag2_storage::MetadataIo metadata_io{};
  rosbag2_storage::BagMetadata metadata{};
  // Specify defaults
  auto info = std::make_shared<rosbag2_cpp::Info>();
  std::shared_ptr<rosbag2_cpp::Reader> reader;
  auto writer = std::make_shared<rosbag2_cpp::Writer>(
    std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
  // Change reader based on metadata options
  if (metadata_io.metadata_file_exists(storage_options.uri)) {
    metadata = metadata_io.read_metadata(storage_options.uri);
    if (metadata.compression_format == "zstd") {
      reader = std::make_shared<rosbag2_cpp::Reader>(
        std::make_unique<rosbag2_compression::SequentialCompressionReader>());
    } else {
      reader = std::make_shared<rosbag2_cpp::Reader>(
        std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    }
  } else {
    reader = std::make_shared<rosbag2_cpp::Reader>(
      std::make_unique<rosbag2_cpp::readers::SequentialReader>());
  }

  rosbag2_transport::Rosbag2Transport transport(reader, writer, info);
  transport.init();
  transport.play(storage_options, play_options);
  transport.shutdown();

  Py_RETURN_NONE;
}

static PyObject *
rosbag2_transport_info(PyObject * Py_UNUSED(self), PyObject * args, PyObject * kwargs)
{
  static const char * kwlist[] = {"uri", "storage_id", nullptr};

  char * char_uri;
  char * char_storage_id;
  if (!PyArg_ParseTupleAndKeywords(
      args, kwargs, "ss", const_cast<char **>(kwlist), &char_uri, &char_storage_id))
  {
    return nullptr;
  }

  std::string uri = std::string(char_uri);
  std::string storage_id = std::string(char_storage_id);

  rosbag2_transport::Rosbag2Transport transport;
  transport.print_bag_info(uri, storage_id);

  Py_RETURN_NONE;
}

/// Define the public methods of this module
#if __GNUC__ >= 8
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wcast-function-type"
#endif
static PyMethodDef rosbag2_transport_methods[] = {
  {
    "record", reinterpret_cast<PyCFunction>(rosbag2_transport_record), METH_VARARGS | METH_KEYWORDS,
    "Record to bag"
  },
  {
    "play", reinterpret_cast<PyCFunction>(rosbag2_transport_play), METH_VARARGS | METH_KEYWORDS,
    "Play bag"
  },
  {
    "info", reinterpret_cast<PyCFunction>(rosbag2_transport_info), METH_VARARGS | METH_KEYWORDS,
    "Print bag info"
  },
  {nullptr, nullptr, 0, nullptr}  /* sentinel */
};
#if __GNUC__ >= 8
# pragma GCC diagnostic pop
#endif

PyDoc_STRVAR(
  rosbag2_transport__doc__,
  "Python module for rosbag2 transport");

/// Define the Python module
static struct PyModuleDef _rosbag2_transport_module = {
  PyModuleDef_HEAD_INIT,
  "_rosbag2_transport",
  rosbag2_transport__doc__,
  -1,   /* -1 means that the module keeps state in global variables */
  rosbag2_transport_methods,
  nullptr,
  nullptr,
  nullptr,
  nullptr
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rosbag2_transport_py(void)
{
  return PyModule_Create(&_rosbag2_transport_module);
}
